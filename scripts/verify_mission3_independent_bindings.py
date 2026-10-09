#!/usr/bin/env python3
"""실제 별도 검토의 파일·평가 결합만 재검사한다. 리뷰/승인을 만들지 않는다."""
import argparse
from datetime import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
from zoneinfo import ZoneInfo


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def verify(review_root, output):
    assert not output.exists(), output
    final_path = review_root / 'final-verification.json'
    index_path = review_root / 'signed-review-index.json'
    final, index = read(final_path), read(index_path)
    for key, name in [('index_sha256', 'signed-review-index.json'),
                      ('own_evidence_sha256', 'own-evidence-sha256.json')]:
        assert sha(review_root / name) == final[key], (key, name)
    protected_checks = []
    for name in ('protected-original-files-sha256.json',
                 'protected-prior-decisions-sha256.json', 'own-evidence-sha256.json'):
        files = read(review_root / name)
        changed = [path for path, digest in files.items() if sha(path) != digest]
        assert not changed, (name, changed)
        protected_checks.append({'manifest': name, 'count': len(files), 'changed': changed,
                                 'manifest_sha256': sha(review_root / name)})
    bindings = []
    for item in index['reviews']:
        run, review_path = Path(item['run']), Path(item['review_path'])
        assert sha(review_path) == item['review_sha256']
        review, manifest = read(review_path), read(run / 'run_manifest.json')
        evaluation_path = run / 'offline-evaluation.json'
        evaluation = read(evaluation_path)
        assert sha(evaluation_path) == review['evaluation_file_sha256'] == item['evaluation_file_sha256']
        evaluator_path = run / 'source-snapshot/scripts/mission3/evaluation.py'
        assert sha(evaluator_path) == review['evaluator_sha256']
        spec = importlib.util.spec_from_file_location('binding_evaluation', evaluator_path)
        evaluator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(evaluator)
        assert evaluator.evaluation_digest(evaluation) == review['evaluation_sha256']
        for key in ('code_sha256', 'runtime_library_sha256'):
            assert review[key] == manifest[key], (item['key'], key)
            for name, digest in review[key].items():
                assert sha(run / 'source-snapshot' / name) == digest, name
        for key in ('config_sha256', 'trial_metadata_sha256'):
            assert review[key] == manifest[key], (item['key'], key)
        for key in ('input_sha256', 'review_evidence_sha256'):
            for name, digest in review[key].items():
                assert sha(name) == digest, name
        valid = evaluator.validate_review(review, evaluation)
        assert valid == item['validation'], (item['key'], valid)
        assert valid['valid'] and not valid['approves_qualified']
        bindings.append({'key': item['key'], 'run_id': manifest['run_id'],
                         'review_sha256': sha(review_path), 'validation': valid,
                         'input_files': len(review['input_sha256']),
                         'evidence_files': len(review['review_evidence_sha256'])})
    result = {'created_kst': datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
              'scope': '별도 담당이 실제 작성한 리뷰의 main SHA 재검사. 새 독립 서명/승인을 생성하지 않음.',
              'reviewer_index_sha256': sha(index_path), 'reviewer_final_sha256': sha(final_path),
              'protected_and_evidence_checks': protected_checks, 'bindings': bindings,
              'candidate_selected': index['candidate_selected'],
              'independently_reviewed_conditionals': sum(b['validation']['approves_simulation_contact'] for b in bindings),
              'qualified': 0, 'validator_sha256': sha(__file__)}
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({key: result[key] for key in ('independently_reviewed_conditionals', 'qualified', 'candidate_selected')}, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('review_root', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    verify(args.review_root, args.out)
