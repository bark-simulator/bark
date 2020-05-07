deps = None
f = open("MANIFEST")
deps = f.readlines()
f.close()

f = open("MANIFEST.in","w")
for dep in deps:
    if '.py' not in dep:
        f.write('include '+ dep.split()[0] + '\n')

f.close()
