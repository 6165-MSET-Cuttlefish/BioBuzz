import os
import sys
import time
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shotcore as sc


def one(name):
    t = time.time()
    sc.load_field(name)
    return name, time.time() - t


if __name__ == "__main__":
    with Pool(4) as p:
        for name, s in p.imap_unordered(one, list(sc.TRUTHS)):
            print(f"{name}: {s:.1f}s", flush=True)
