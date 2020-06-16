output="PF1 "

for n in range(16):
    line="%"
    for i in range(3, -1, -1):
        if n & (2**i) != 0:
            line += "11"
        else:
            line += "00"
    output += line + ","

print output
print

output="PF2 "

for n in range(16):
    line="%"
    for i in range(4):
        if n & (2**i) != 0:
            line += "11"
        else:
            line += "00"
    output += line + ","

print output
print

output="PF0 "

for n in range(16):
    line="%"
    for i in range(2,4):
        if n & (2**i) != 0:
            line += "11"
        else:
            line += "00"
    output += line + "0000" + ","

print output
print

output="PF1 (left) "

for n in range(16):
    line="%"
    for i in range(1, -1, -1):
        if n & (2**i) != 0:
            line += "11"
        else:
            line += "00"
    output += line + "0000" + ","

print output
print

output="PF1 (right) "

for n in range(16):
    line=""
    for i in range(3, 1, -1):
        if n & (2**i) != 0:
            line += "11"
        else:
            line += "00"
    output += "%0000" + line + ","

print output
print

output="PF2 "

for n in range(16):
    line=""
    for i in range(2):
        if n & (2**i) != 0:
            line += "11"
        else:
            line += "00"
    output += "%0001" + line + ","

print output
print

output="NEW %"

for n in range(16):
    line=""
    for i in range(4):
        if n & (2**i) != 0:
            line += "1"
        else:
            line += "0"
    output += line + "1000,%"

print output
print
