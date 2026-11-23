"""Print every global a .pt checkpoint references, without unpickling it."""
import pickletools
import sys
import zipfile

for path in sys.argv[1:]:
    with zipfile.ZipFile(path) as archive:
        data = archive.read(next(n for n in archive.namelist() if n.endswith("data.pkl")))
    found = {arg.replace(" ", ".") for op, arg, _ in pickletools.genops(data) if op.name == "GLOBAL"}
    print(path)
    for name in sorted(found):
        print(" ", name)
