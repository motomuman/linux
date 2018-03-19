
lklconfig = set()
miniconfig = set()

for line in open(".config", "r"):
    line = line.rstrip("\n")
    if "=y" in line:
        lklconfig.add(line)

for line in open("./miniconfig", "r"):
    line = line.rstrip("\n")
    if "=y" in line:
        miniconfig.add(line)


for c in lklconfig:
    if c not in miniconfig:
        print "lkl has ", c

print ""
print ""

for c in miniconfig:
    if c not in lklconfig:
        print "mini has ", c
