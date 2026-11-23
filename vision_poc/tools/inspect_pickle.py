"""List the globals a .pt checkpoint imports, without unpickling it."""
import pickletools
import sys
import zipfile

for path in sys.argv[1:]:
    with zipfile.ZipFile(path) as archive:
        name = next(n for n in archive.namelist() if n.endswith("data.pkl"))
        data = archive.read(name)
    found, strings = set(), []
    for op, arg, _ in pickletools.genops(data):
        if op.name in ("SHORT_BINUNICODE", "BINUNICODE", "UNICODE"):
            strings.append(arg)
        elif op.name == "GLOBAL":
            found.add(arg.replace(" ", "."))
        elif op.name == "STACK_GLOBAL":
            found.add(".".join(strings[-2:]))
    print(path, len(found))
    print("  modules:", sorted({g.split(".")[0] for g in found}))
    print("  suspicious:", sorted(g for g in found if g.split(".")[0] in
          ("os", "posix", "nt", "subprocess", "sys", "socket", "shutil", "builtins", "runpy", "importlib")))
