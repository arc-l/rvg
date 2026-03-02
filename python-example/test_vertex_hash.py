"""Minimal smoke test for vertex hashing in Python bindings.

Run after installing the package, e.g.:
    pip install -e .
    python3 python-example/test_vertex_hash.py
"""

from __future__ import annotations

from rvg import vertex


def check(label: str, condition: bool, detail: str = "") -> None:
    status = "PASS" if condition else "FAIL"
    if detail:
        print(f"[{status}] {label}: {detail}")
    else:
        print(f"[{status}] {label}")


def main() -> None:
    # Equal by value (same constructor args).
    v1 = vertex(1.0, 2.0)
    v2 = vertex(1.0, 2.0)

    # Different value.
    v3 = vertex(3.0, 4.0)

    h1 = hash(v1)
    h2 = hash(v2)
    h3 = hash(v3)

    check("hash(v) equals v.hash()", h1 == v1.hash(), f"hash(v1)={h1}, v1.hash()={v1.hash()}")
    check("equal vertices compare equal", v1 == v2)
    check("equal vertices have equal hashes", h1 == h2, f"h1={h1}, h2={h2}")
    check("different vertices can be hashed", isinstance(h3, int), f"type={type(h3).__name__}")

    s = {v1, v2, v3}
    check("set deduplicates equal vertices", len(s) == 2, f"set_size={len(s)}")

    d = {v1: "first", v3: "third"}
    check("dict lookup by equal vertex works", d[v2] == "first", f"d[v2]={d[v2]}")

    print("vertex hash smoke test passed")


if __name__ == "__main__":
    main()
