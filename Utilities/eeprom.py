
from __future__ import print_function
import argparse
import struct
import re
import binascii
import serial
import time
import sys

parser = argparse.ArgumentParser(description = "AQ32Plus eeprom tool")

parser.add_argument('input', nargs='?', type=argparse.FileType('r'),
	help="serial dump including output from the 'd' command of the EEPROM CLI " +
	     "('U' from main CLI); this doesn't have to be JUST the hex dump -- " +
	     "the important thing is to not mess with the hex dump formatting and " +
	     "ensure it is bounded by newline characters")
parser.add_argument('--raw', nargs='?', type=argparse.FileType('wb'),
	help="output the eepromConfig_t struct in binary, to this file")
parser.add_argument('--exclude-crc', action='store_true',
	help="exclude the CRC when writing to file (use with --raw)")
parser.add_argument('--port', nargs='?',
	help="serial port name for uploading/downloading")
parser.add_argument('--baud', type=int, default=115200,
	help="baud rate for serial port")
parser.add_argument('-c', nargs='?', dest='download', type=argparse.FileType('wb'),
	help="download eeprom config to file (see 'c' command in EEPROM CLI)")
parser.add_argument('-C', nargs='?', dest='upload', type=argparse.FileType('rb'),
	help="upload file to in-memory eeprom config (see 'C' command in EEPROM CLI)")
parser.add_argument('-q', action='store_true', dest='quiet',
	help="output no text, just return 0 for success and nonzero otherwise")
parser.add_argument('-v', action='store_true', dest='verbose',
	help="verbose output (see serial communication)")

args = parser.parse_args()

def parse_eeprom_dump(contents):
	m = re.search(r"^([A-Za-z0-9]{16,}(\n\r?|\r\n?))+", contents, re.M | re.X)

	if not m:
		raise Exception("Could not find eeprom dump.")

	eeprom = binascii.unhexlify(m.group(0).replace('\n', ''))
	# CRC32B should be the last four bytes
	payload  = eeprom[:-4]
	(crc32b,) = struct.unpack('<I', eeprom[-4:])
	computed_crc32b = (binascii.crc32(payload, 0) & 0xFFFFFFFF)

	return (eeprom, payload, crc32b, computed_crc32b)

# tile('abcdefg', 3) returns ['abc', 'def', 'g']
def tile(s, sz):
	return [s[i*sz:(i+1)*sz] for i in xrange((len(s)+sz-1)/sz)]

def error_and_exit(message):
	print(message, file=sys.stderr)
	exit(1)

if args.input:
	try:
		(eeprom, payload, crc32b, computed_crc32b) = parse_eeprom_dump(args.input.read())
	except Exception as ex:
		error_and_exit(ex)

	crcs_match = crc32b == computed_crc32b

	if args.raw:
		args.raw.write(payload if args.exclude_crc else eeprom)
		args.raw.close()

	if crcs_match:
		if not args.quiet:
			print('CRCs match! (%08x)' % crc32b)
	else:
		if not args.quiet:
			print('CRC mismatch!', file=sys.stderr)
			print('%08x <-- CRC32B from eeprom dump'   % crc32b, file=sys.stderr)
			print('%08x <-- CRC32B from recomputation' % computed_crc32b, file=sys.stderr)
		exit(1)


if args.port and (args.upload or args.download):
	try:
		ser = None # otherwise we get NameError in the finally statement if the port is invalid
		ser = serial.Serial(args.port, args.baud, timeout = 0.100)
		ser.flushInput()
		ser.flushOutput()

		t0 = time.time()

		def write_ser(s):
			if args.verbose:
				print('%.3f < %s' % (time.time() - t0, s))
			ser.write(s)
		def ser_read_with_timeout(len, timeout):
			old_timeout = ser.timeout
			ser.timeout = timeout
			by = ser.read(len)
			ser.timeout = old_timeout
			return by
		def ser_readline_stripped():
			return ser.readline().rstrip('\n')
		def print_timestamped(prefix, by):
			print("%.3f %s> %s" % (time.time() - t0, prefix, by))


		write_ser('x') # any CLI command: exit whatever sub-CLI we may be in (otherwise 'x' does nothing)
		time.sleep(0.010)
		ser.flushInput()
		write_ser('U') # main CLI command: enter EEPROM CLI

		by = ser_read_with_timeout(1000, 0.300)

		if not re.search(r"^Entering EEPROM CLI[.]{4}$", by, re.M):
			error = "Didn't get expected CLI response; ensure CLI is operational then try again.\n ->"
			if len(by) > 0:
				error += "This is what was received: '%s'" % by
			else:
				error += "Nothing received over serial line."
			error_and_exit(error)

		if args.download:
			write_ser('c') # EEPROM CLI command: dump in-memory eeprom contents
			by = ser_read_with_timeout(10000, 0.300)
			if len(by) > 0:
				if args.verbose:
					print(by)

				try:
					(eeprom, payload, crc32b, computed_crc32b) = parse_eeprom_dump(by)
				except Exception:
					error_and_exit("Could not parse eeprom dump!")

				if crc32b != computed_crc32b:
					error_and_exit("CRC mismatch!")
				else:
					args.download.write("%s\n" % binascii.hexlify(eeprom))
					if not args.quiet:
						print("CRC match; file written.")
					time.sleep(0.500)
					write_ser('x')
					ser.flushOutput()
			else:
				error_and_exit("No dump received!")

		if args.upload:
			(eeprom, payload, crc32b, computed_crc32b) = parse_eeprom_dump(args.upload.read())

			if crc32b != computed_crc32b:
				error_and_exit("CRC mismatch!")

			write_ser('C') # EEPROM CLI command: upload eeprom as hexadecimal

			# four lines are sent if the 'C' command were processed properly
			t = time.time()
			while time.time() - t < 0.200:
				by = ser_readline_stripped()
				if args.verbose:
					print_timestamped('1', by)
				if re.search(r"Times out if no character", by):
					break

			if len(by) == 0:
				error_and_exit("Did not get expected CLI response from 'C' command!")
			else:
				for s in tile(binascii.hexlify(eeprom), 64):
					if ser.inWaiting() != 0: # nothing should be recieved during this time period
						break
					write_ser(s)
					time.sleep(0.001) # give a little time to the serial interface
					
				# this prints what the EEPROM CLI reports about the uploaded config
				t = time.time()
				while time.time() - t < 0.200:
					by = ser_readline_stripped()
					if len(by) > 0 and not re.search(r"^EEPROM CLI ->\s*$", by):
						print_timestamped('2', by)

				t = time.time()
				write_ser('x')
				while time.time() - t < 0.200:
					by = ser_readline_stripped()
					if len(by) > 0 and args.verbose:
						print_timestamped('3', by)

	except serial.SerialException as ex:
		print(ex, file=sys.stderr)

	# except Exception as ex:
	# 	raise

	except (SystemExit, KeyboardInterrupt):
		pass

	finally:
		if ser and ser.isOpen():
			ser.close()

