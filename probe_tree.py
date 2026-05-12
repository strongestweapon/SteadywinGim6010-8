"""
probe_tree.py
펌웨어 v0.6.5 보드의 객체 트리를 dump. 1회성 도구.
2차 인코더가 어디 매달려있는지, pos_estimate/vel_estimate 의 정확한 경로를 찾기 위함.

사용법:
    python probe_tree.py > tree.txt
"""
import sys
import odrive

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


def safe_get(obj, name):
    try:
        return getattr(obj, name)
    except Exception as e:
        return f"<err: {e}>"


def attr_names(obj):
    """RemoteObject 의 user-visible 속성 이름 (밑줄 제외)."""
    out = []
    for n in dir(obj):
        if n.startswith("_"):
            continue
        out.append(n)
    return out


def dump_object(obj, name, depth=0, max_depth=3, seen=None):
    """객체 트리를 재귀 dump. 단순 스칼라는 값 표시, child object 는 재귀."""
    if seen is None:
        seen = set()
    pad = "  " * depth
    oid = id(obj)
    if oid in seen:
        print(f"{pad}{name}: <순환>")
        return
    seen.add(oid)

    # 스칼라 / primitive 표시
    if isinstance(obj, (int, float, bool, str, bytes)) or obj is None:
        print(f"{pad}{name} = {obj!r}")
        return

    names = attr_names(obj)
    # RemoteObject 가 아닌 일반 리스트/딕트 등이면 그냥 표시
    if not names:
        print(f"{pad}{name} = {obj!r}")
        return

    print(f"{pad}{name}/")
    if depth >= max_depth:
        print(f"{pad}  ... (max_depth)")
        return

    for n in names:
        v = safe_get(obj, n)
        # 호출 가능(메소드) 은 표기만
        if callable(v) and not hasattr(v, "_codecs"):
            print(f"{pad}  {n}()  <callable>")
            continue
        # 스칼라
        if isinstance(v, (int, float, bool, str, bytes)) or v is None:
            print(f"{pad}  {n} = {v!r}")
        else:
            # 자식 객체 — 재귀
            dump_object(v, n, depth + 1, max_depth, seen)


def main() -> int:
    print("ODrive 연결 중...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"연결 실패: {e}")
        return 1

    print("=" * 60)
    print("최상위 (odrv0) 직속 children")
    print("=" * 60)
    for n in attr_names(odrv):
        v = safe_get(odrv, n)
        if isinstance(v, (int, float, bool, str)) or v is None:
            print(f"  {n} = {v!r}")
        elif callable(v) and not hasattr(v, "_codecs"):
            print(f"  {n}()  <callable>")
        else:
            print(f"  {n}/   <object>")

    print()
    print("=" * 60)
    print("axis0 트리 (depth=3)")
    print("=" * 60)
    dump_object(odrv.axis0, "axis0", max_depth=3)

    # 2차 인코더 후보 노드 — 위치/속도 같이 출력
    print()
    print("=" * 60)
    print("인코더 후보 노드들의 pos_estimate / pos_rel / vel_estimate")
    print("=" * 60)
    candidates = []
    for n in attr_names(odrv):
        if "encoder" in n.lower() or "mapper" in n.lower():
            candidates.append(("odrv0." + n, safe_get(odrv, n)))
    # axis0 안의 mapper 도
    for n in attr_names(odrv.axis0):
        if "mapper" in n.lower() or "encoder" in n.lower():
            candidates.append(("odrv0.axis0." + n, safe_get(odrv.axis0, n)))

    if not candidates:
        print("  (없음 — axis0 직속의 pos_estimate / vel_estimate 만 사용)")
    for path, obj in candidates:
        print(f"\n  [{path}]")
        for field in ("pos_estimate", "pos_rel", "pos_abs",
                      "vel_estimate", "shadow_count", "count_in_cpr",
                      "is_ready"):
            if hasattr(obj, field):
                try:
                    v = getattr(obj, field)
                    print(f"    {field} = {v!r}")
                except Exception as e:
                    print(f"    {field} = <err: {e}>")

    # axis0 직속의 pos_estimate / vel_estimate
    print("\n  [odrv0.axis0 직속]")
    for field in ("pos_estimate", "vel_estimate"):
        if hasattr(odrv.axis0, field):
            try:
                v = getattr(odrv.axis0, field)
                print(f"    {field} = {v!r}")
            except Exception as e:
                print(f"    {field} = <err: {e}>")

    return 0


if __name__ == "__main__":
    sys.exit(main())
