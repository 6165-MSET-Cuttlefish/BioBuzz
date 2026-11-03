import builtins
import pathlib
import sys
import symtable


def walk(table, module_names, out):
    for sym in table.get_symbols():
        if (sym.is_global() or (table.get_type() == "module" and sym.is_referenced())) and sym.is_referenced():
            name = sym.get_name()
            if name not in module_names and not hasattr(builtins, name):
                out.add((table.get_name(), table.get_lineno(), name))
    for child in table.get_children():
        walk(child, module_names, out)


def main(path):
    src = open(path).read()
    top = symtable.symtable(src, path, "exec")
    module_names = {s.get_name() for s in top.get_symbols() if s.is_assigned() or s.is_imported() or s.is_namespace()}
    module_names.add("__file__")
    out = set()
    walk(top, module_names, out)
    for scope, line, name in sorted(out, key=lambda x: (x[1], x[2])):
        print(f"{path}: scope {scope} (line {line}) uses undefined {name}")
    print(f"undefined names: {len(out) or 'none'}")
    return 1 if out else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else
                  str(pathlib.Path(__file__).resolve().parents[3] / "scripts/bench/run_bench.py")))
