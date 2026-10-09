#!/usr/bin/env python3
"""미션3 실행 종료 후 평가 CLI. 출력은 제어기·추정기·plant의 입력으로 쓰지 않는다.

  python scripts/evaluate_mission3.py run RUN_DIR [--out FILE]
  python scripts/evaluate_mission3.py review-template RUN_DIR --reviewer ID [--out FILE]
  python scripts/evaluate_mission3.py ledger ATTEMPTS.json [--out FILE]

ATTEMPTS.json은 실행 순서대로 [{"run_dir": "...", "reviews": ["review.json", ...]}, ...]다.
상대 경로는 ATTEMPTS.json이 있는 디렉터리 기준이다. ledger는 저장된 평가 파일을 믿지 않고
run_dir 로그를 다시 평가한 뒤 리뷰 해시와 대조한다. 계약: docs/mission3/evaluation-contract.md
"""
import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mission3.evaluation import build_ledger, evaluate_run, review_template  # noqa: E402
from mission3.settings import ROOT  # noqa: E402


def canonical_run(path):
    """Stable informational run_dir for the same project, independent of cwd.

    Historical signatures use paths relative to ROOT. Resolve before changing
    cwd and pass that canonical spelling to the unchanged evaluator.
    """
    resolved=Path(path).resolve()
    try:canonical=resolved.relative_to(ROOT)
    except ValueError:canonical=resolved
    os.chdir(ROOT)
    return evaluate_run(canonical)


def emit(payload, out):
    text = json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
    if out is None:
        sys.stdout.write(text)
        return
    out = Path(out)
    if out.exists():
        raise SystemExit(f"기존 기록을 덮어쓰지 않습니다: {out}")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="utf-8")
    print(f"저장 {out}", file=sys.stderr)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="완료된 실행 하나를 평가")
    run.add_argument("run_dir", type=Path)
    run.add_argument("--out", type=Path)
    template = sub.add_parser("review-template", help="서명 전 리뷰 양식(승인 플래그 모두 False)")
    template.add_argument("run_dir", type=Path)
    template.add_argument("--reviewer", required=True)
    template.add_argument("--out", type=Path)
    ledger = sub.add_parser("ledger", help="전체 시도와 진입점 진행 집계")
    ledger.add_argument("attempts", type=Path)
    ledger.add_argument("--out", type=Path)
    args = parser.parse_args(argv)

    if args.command == "run":
        # Preserve the output target interpretation before canonical cwd.
        out=None if args.out is None else args.out.resolve()
        emit(canonical_run(args.run_dir),out)
    elif args.command == "review-template":
        out=None if args.out is None else args.out.resolve()
        emit(review_template(canonical_run(args.run_dir), args.reviewer),out)
    else:
        base = args.attempts.resolve().parent
        out=None if args.out is None else args.out.resolve()
        items=json.loads(args.attempts.read_text(encoding="utf-8"))
        attempts = []
        for item in items:
            run_dir = (base / item["run_dir"]).resolve()
            reviews = [json.loads((base / path).read_text(encoding="utf-8")) for path in item.get("reviews", [])]
            attempts.append({"evaluation": canonical_run(run_dir), "reviews": reviews})
        emit(build_ledger(attempts),out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
