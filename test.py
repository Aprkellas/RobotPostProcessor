import re

def removeExternalAxis(input_file, output_file):
  with open(input_file, 'r') as infile, open (output_file, 'w+') as outfile:
    previous_line = ""

    for line in infile:
            # Check if the line contains "E1" followed by a value
            match = re.search(r'E1\s*=\s*[\d.]+\s*mm', line)

            if match and previous_line.strip().endswith(','):
                for line in outfile:
                    if line == previous_line:
                        line.replace('')
                previous_line = previous_line.rstrip(',\n')
                outfile.write(previous_line)
                

            if match:
                # Replace the matched part with an empty string
                line = line.replace(match.group(0), '')

            # Write the modified line to the output file
            outfile.write(line)
            previous_line = line

input = "./MAIN - Copy.LS"
output = "./MAIN - mod.LS"
removeExternalAxis(input, output)