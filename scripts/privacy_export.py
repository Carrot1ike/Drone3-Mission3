#!/usr/bin/env python3
"""공유용 파생 자료 생성. 실험 원본을 덮어쓰지 않는다."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import subprocess
import tempfile
import zipfile

REPO = 'Carrot1ike/Drone3-Mission3'
TAG = 'mission3-straight-20261009'
BASE = '069aa317c81dd58b7633cbf178282cf9d1e39e24'
OLD_TAG = '82944a22f9655019ee28bc967f2f845acde4240d'
ASSETS = [
    ('mission3-team-share-privacy-20261010.zip', 'f3d757d8fccbe0487caade5c27d9ea183894b8cafe68ef2597f5dbfa1f08e2ba', 'mission3-team-share-privacy-20261010.zip'),
    ('mission3-straight-16-privacy-20261010.zip', '159c70fbc3ac31cbb4a4d2f4c9fd4a9fb9a40c472508e6acc135430dd5d21d2a', 'mission3-straight-16-privacy-20261010.zip'),
]
ROOT = re.compile(r'/home/[A-Za-z0-9_.-]+/(?:Desktop|Documents/ChatGPT)/Drone3')
HOME_PATH = re.compile(r'/home/[A-Za-z0-9_.-]+')
PRIVATE_PATH = re.compile(rb'/home/(?!runner(?:/|\b))[A-Za-z0-9_.-]+/')
DROP = {'docs/mission3/session-stop-20261009.md'}
LAUNCHER = '''#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/mission3_runtime.bash"
cd "$mission3_root"
exec "$mission3_python" "$mission3_root/scripts/run_mission3.py" "$@"
'''
NOTE = '''# 공유 자료의 개인정보 정리 — 2026-10-10

커밋의 작성자·커미터는 프로젝트용 noreply 식별자로 익명화했습니다.
문서의 개인 PC 경로는 상대 경로로, 원로그의 개인 경로는 익명 작업 경로로 변경했습니다.
개인 컴퓨터 종료·예약 실행 기록은 공유 범위에서 제외했습니다.
과거 제어·센서·검토 담당 식별자는 기술적 검증 근거이며 실제 사용자 실명이라고 주장하지 않습니다.

Release의 두 ZIP은 **개인정보를 제거한 파생 공유본**입니다. ZIP 안의
`PRIVACY_EXPORT.json`에 원 ZIP SHA256, 수정·제외한 상대 파일명과 새 파일 SHA256을 기록했습니다.
개인 경로가 들어가는 호스트 빌드 `.so`는 제외했습니다. 팀원은 현재 main의
`scripts/setup_mission3.sh`로 자기 PC에서 라이브러리를 다시 빌드합니다.
영상·이미지·PDF는 바이트를 유지했고, JSON/JSONL의 측정 수치·제어 명령은 변경하지 않았습니다.
과거 source/파일 해시는 **익명화 전 원자료**에 대한 기록입니다.
익명화된 파일에 원 byteexact 인증을 그대로 적용하지 마세요.
시험 결과는 같은 seed의 개발16회 중 조건부 완료14회/실패2회이며 정식·실기 승인은0입니다.

개인정보 제거로 Git 커밋 ID가 변경됐습니다. 기존 clone을 쓰는 팀원은
개인 수정 사항을 따로 보관한 뒤 새 폴더에 다시 clone하고, 과거 이력을 다시 push하지 마세요.
이미 받은 ZIP과 clone은 소유자가 직접 제거해야 합니다. 서버의 예전 SHA 캐시까지 제거됐다고
주장하지 않습니다. 필요시 GitHub 지원에 별도 삭제 요청이 필요할 수 있습니다.

새 커밋은 GitHub의 noreply 이메일을 사용하고, 개인 인증 파일·PC 경로·대화 기록을 공유하지 마세요.
새 배포 전에는 `python3 scripts/privacy_export.py scan <공유 폴더>`로 홈 경로를 검사할 수 있습니다.
'''
EXPORT_NOTE = '''이 ZIP은 2026-10-10 개인정보를 제거한 파생 공유본입니다.
PRIVACY_EXPORT.json을 먼저 읽으세요. 원 ZIP과 동일한 파일 해시를 주장하지 않습니다.
개인 경로를 익명화하고 머신별 빌드 .so를 제외했습니다. 영상·영상 픽셀·측정 수치는 유지했습니다.
라이브러리는 저장소 main의 설치기로 다시 빌드하세요. 원16회 결과와 분모는 변경하지 않았습니다.
'''

def sha(data):
    return hashlib.sha256(data).hexdigest()

def file_sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

def sanitize(path, data, archive=False):
    try:
        text = data.decode('utf-8')
    except UnicodeDecodeError:
        if PRIVATE_PATH.search(data):
            raise ValueError('Private path remains in a binary file: ' + path)
        return data
    if not archive and path == 'scripts/run_mission3.sh':
        return LAUNCHER.encode()
    if path.endswith('.md') and not archive:
        # GitHub 문서 링크를 해당 문서 디렉터리 기준 상대 경로로 바꾼다.
        def link(m):
            target = ROOT.sub('', m.group(1), count=1).lstrip('/')
            return '](' + os.path.relpath(target, str(PurePosixPath(path).parent)) + ')'
        text = re.sub(r'\]\((/home/[^)]+/(?:Desktop|Documents/ChatGPT)/Drone3/[^)]+)\)', link, text)
    text = ROOT.sub('./Drone3' if archive else '.', text)
    text = HOME_PATH.sub('/workspace', text)
    if not archive:
        if path == 'README.md':
            text = re.sub(r'^문서에 남은 .*절대 경로.*\n', '', text, flags=re.M)
            if 'privacy-sharing-20261010.md' not in text:
                text += '\n공유 자료의 경로·커밋 메타데이터와 Release ZIP은 [개인정보 정리 안내](docs/mission3/privacy-sharing-20261010.md)를 참고하세요.\n'
        if path == 'docs/mission3/session-report-20261009.md':
            text = text.replace('2026-10-09 09:00 KST 마감. ', '2026-10-09 개발 결과. ')
            text = re.sub(r'^09:00 마감 후.*(?:\n|$)', '', text, flags=re.M)
        if path == 'docs/mission3/entry-hold-35x12.md':
            text = re.sub(r'^사용자가10월9일09:00.*(?:\n|$)', '', text, flags=re.M)
        if path == 'docs/git-sharing.md':
            text = text.replace('사용자는 새 미션3 저장소를 선택하고 자신의 계정 업로드를 승인했다. 대상은 ', '공유 대상은 ')
        for old, _, new in ASSETS:
            text = text.replace(old, new)
    return text.encode()

def check_numbers(before, after, suffix):
    def compare(a, b):
        if isinstance(a, dict):
            assert len(a) == len(b)
            mapped_keys = []
            for key in a:
                mapped = HOME_PATH.sub('/workspace', ROOT.sub('./Drone3', key))
                mapped_keys.append(mapped)
                assert mapped in b, 'Non-path JSON key changed'
                compare(a[key], b[mapped])
            assert len(set(mapped_keys)) == len(mapped_keys), 'Redaction key collision'
        elif isinstance(a, list):
            assert len(a) == len(b)
            for x, y in zip(a, b):
                compare(x, y)
        elif not isinstance(a, str):
            assert a == b, 'Non-string experiment value changed'
        else:
            assert isinstance(b, str)
    if suffix == '.json':
        compare(json.loads(before), json.loads(after))
    elif suffix == '.jsonl':
        aa, bb = before.splitlines(), after.splitlines()
        assert len(aa) == len(bb)
        for a, b in zip(aa, bb):
            if a.strip():
                compare(json.loads(a), json.loads(b))

def export_archive(source, output):
    original = file_sha(source)
    expected = next(d for n, d, _ in ASSETS if Path(source).name == n)
    assert original == expected, 'Original release digest changed'
    records, omitted, files, media = [], [], 0, 0
    with zipfile.ZipFile(source) as src, zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as dst:
        for info in src.infolist():
            if info.is_dir():
                continue
            assert not info.filename.startswith('/') and '..' not in PurePosixPath(info.filename).parts
            if info.filename.endswith('.so'):
                omitted.append({'path': info.filename, 'reason': 'machine-specific compiled library; rebuild using setup_mission3.sh'})
                continue
            data = src.read(info)
            new = sanitize(info.filename, data, archive=True)
            if new != data:
                check_numbers(data, new, PurePosixPath(info.filename).suffix)
                records.append({'path': info.filename, 'original_sha256': sha(data), 'export_sha256': sha(new)})
            assert not PRIVATE_PATH.search(new), 'Private home path remains'
            if PurePosixPath(info.filename).suffix.lower() in {'.mp4', '.avi', '.png', '.jpg', '.pdf'}:
                assert data == new
                media += 1
            clean_info = zipfile.ZipInfo(info.filename, info.date_time)
            clean_info.external_attr = info.external_attr
            clean_info.compress_type = zipfile.ZIP_DEFLATED
            dst.writestr(clean_info, new, compresslevel=6)
            files += 1
        manifest = {'format': 'privacy-derived-export-v1', 'date': '2026-10-10', 'original_archive_sha256': original,
                    'retained_files': files, 'unchanged_media_files': media, 'modified_files': records,
                    'excluded_files': omitted, 'numerical_json_values_preserved': True,
                    'old_source_signatures_apply_to_original_only': True, 'conditional_completed': 14, 'failed': 2,
                    'formal_hardware_qualified': 0}
        for name, content in [('PRIVACY_EXPORT.json', json.dumps(manifest, ensure_ascii=False, indent=2) + '\n'),
                              ('PRIVACY_README.txt', EXPORT_NOTE)]:
            info = zipfile.ZipInfo(name, (2026, 10, 10, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            dst.writestr(info, content.encode(), compresslevel=6)
    return {'filename': Path(output).name, 'bytes': Path(output).stat().st_size, 'sha256': file_sha(output),
            'modified_files': len(records), 'excluded_libraries': len(omitted), 'unchanged_media_files': media}

def export_directory(source, dest):
    changes = []
    for f in sorted(Path(source).rglob('*')):
        if not f.is_file() or '.git' in f.relative_to(source).parts:
            continue
        rel = f.relative_to(source).as_posix()
        if rel in DROP:
            changes.append({'path': rel, 'change': 'excluded personal session record'})
            continue
        original = f.read_bytes()
        new = sanitize(rel, original)
        target = Path(dest) / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(new)
        target.chmod(f.stat().st_mode & 0o777)
        assert not PRIVATE_PATH.search(new)
        if new != original:
            if rel.endswith(('.json', '.jsonl')) and not rel.endswith('release-assets.json'):
                check_numbers(original, new, Path(rel).suffix)
            changes.append({'path': rel, 'original_sha256': sha(original), 'export_sha256': sha(new)})
    target = Path(dest) / 'docs/mission3/privacy-sharing-20261010.md'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(NOTE)
    return changes

def scan(root):
    bad = []
    for f in Path(root).rglob('*'):
        if f.is_file() and '.git' not in f.relative_to(root).parts and PRIVATE_PATH.search(f.read_bytes()):
            bad.append(f.relative_to(root).as_posix())
    print(json.dumps({'private_home_path_files': bad}, ensure_ascii=False))
    assert not bad

def git(*args, data=None, env=None):
    r = subprocess.run(['git', *args], input=data, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
    if r.returncode:
        raise RuntimeError('Git operation failed: ' + args[0])
    return r.stdout

def rewrite_history(assets, expected_head):
    assert git('rev-parse', 'HEAD').decode().strip() == expected_head
    assert git('rev-parse', 'HEAD^').decode().strip() == BASE
    assert git('rev-parse', 'refs/tags/' + TAG).decode().strip() == OLD_TAG
    commits = git('rev-list', '--reverse', expected_head).decode().splitlines()
    assert len(commits) == 5
    mapped, changed = {}, 0
    for commit in commits:
        meta = git('show', '-s', '--format=%P%x00%aI%x00%cI%x00%B', commit).decode().split('\0', 3)
        parents, author_date, committer_date, message = meta
        with tempfile.TemporaryDirectory() as tmp:
            env = dict(os.environ, GIT_INDEX_FILE=str(Path(tmp) / 'index'),
                       GIT_AUTHOR_NAME='Mission3 Project', GIT_COMMITTER_NAME='Mission3 Project',
                       GIT_AUTHOR_EMAIL='mission3@users.noreply.github.com', GIT_COMMITTER_EMAIL='mission3@users.noreply.github.com',
                       GIT_AUTHOR_DATE=author_date, GIT_COMMITTER_DATE=committer_date)
            git('read-tree', '--empty', env=env)
            entries = []
            for row in git('ls-tree', '-rz', commit).split(b'\0'):
                if not row:
                    continue
                header, name = row.split(b'\t', 1)
                mode, kind, blob = header.decode().split()
                path = name.decode()
                if path in DROP:
                    continue
                assert kind == 'blob'
                old = git('cat-file', 'blob', blob)
                new = sanitize(path, old)
                assert not PRIVATE_PATH.search(new)
                if new != old:
                    blob = git('hash-object', '-w', '--stdin', data=new).decode().strip()
                    changed += 1
                entries.append(f'{mode} {blob}\t{path}\0'.encode())
            if commit == expected_head:
                supplemental = {
                    'docs/mission3/privacy-sharing-20261010.md': NOTE.encode(),
                    'results/mission3/straight-20261009/release-assets.json': (json.dumps({
                        'scope': 'Privacy-derived exports; verified SHA256; old signatures apply to original only',
                        'assets': [dict(a, download_url=f'https://github.com/{REPO}/releases/download/{TAG}/{a["filename"]}',
                                        upload_state='uploaded', original_archive_sha256=ASSETS[i][1]) for i, a in enumerate(assets)],
                        'release_url': f'https://github.com/{REPO}/releases/tag/{TAG}', 'privacy_export_date': '2026-10-10'
                    }, ensure_ascii=False, indent=2) + '\n').encode()}
                entries = [e for e in entries if e.split(b'\t', 1)[1][:-1].decode() not in supplemental]
                for path, data in supplemental.items():
                    blob = git('hash-object', '-w', '--stdin', data=data).decode().strip()
                    entries.append(f'100644 {blob}\t{path}\0'.encode())
            git('update-index', '-z', '--index-info', data=b''.join(entries), env=env)
            tree = git('write-tree', env=env).decode().strip()
            parent_args = sum((['-p', mapped[p]] for p in parents.split()), [])
            mapped[commit] = git('commit-tree', tree, *parent_args, data=message.encode(), env=env).decode().strip()
    return mapped, changed

def publish():
    assert os.environ.get('GITHUB_ACTIONS') == 'true', 'Publishing only in approved repository workflow'
    assert os.environ.get('GITHUB_REPOSITORY') == REPO
    observed = git('rev-parse', 'HEAD').decode().strip()
    assert git('rev-parse', 'HEAD^').decode().strip() == BASE
    assert subprocess.check_output(['gh', 'api', f'repos/{REPO}/git/ref/heads/main', '--jq', '.object.sha']).decode().strip() == observed
    work = Path(tempfile.mkdtemp(prefix='mission3-privacy-'))
    release = json.loads(subprocess.check_output(['gh', 'api', f'repos/{REPO}/releases/tags/{TAG}']))
    assert len(release['assets']) == 2
    exports = []
    for old, digest, new in ASSETS:
        asset = next(a for a in release['assets'] if a['name'] == old)
        assert asset['digest'] == 'sha256:' + digest
        subprocess.run(['gh', 'release', 'download', TAG, '--repo', REPO, '--pattern', old, '--dir', str(work)], check=True)
        exports.append(export_archive(work / old, work / new))
    mapped, changed = rewrite_history(exports, observed)
    # New assets are uploaded and verified before any original attachment is removed.
    for a in exports:
        subprocess.run(['gh', 'release', 'upload', TAG, str(work / a['filename']), '--repo', REPO], check=True)
    new_release = json.loads(subprocess.check_output(['gh', 'api', f'repos/{REPO}/releases/tags/{TAG}']))
    for a in exports:
        uploaded = next(x for x in new_release['assets'] if x['name'] == a['filename'])
        assert uploaded['digest'] == 'sha256:' + a['sha256'] and uploaded['size'] == a['bytes']
    git('push', '--atomic', '--force-with-lease=refs/heads/main:' + observed,
        '--force-with-lease=refs/tags/' + TAG + ':' + OLD_TAG, 'origin',
        mapped[observed] + ':refs/heads/main', mapped[OLD_TAG] + ':refs/tags/' + TAG)
    notes = work / 'release-notes.md'
    notes.write_text((release.get('body') or '') + '\n\n## 2026-10-10 개인정보 정리\n\n'
                     + '기존 첨부는 개인정보 제거 파생 ZIP으로 교체했습니다. PRIVACY_EXPORT.json을 확인하세요. '
                     + '영상·측정값·개발16회(조건부14/실패2)는 유지했습니다. 머신별 .so는 설치기로 재빌드합니다. '
                     + 'Git 이력이 변경됐으므로 기존 clone 대신 새 clone을 사용하세요.\n')
    subprocess.run(['gh', 'release', 'edit', TAG, '--repo', REPO, '--notes-file', str(notes)], check=True)
    for old, _, _ in ASSETS:
        subprocess.run(['gh', 'release', 'delete-asset', TAG, old, '--repo', REPO, '--yes'], check=True)
    final_release = json.loads(subprocess.check_output(['gh', 'api', f'repos/{REPO}/releases/tags/{TAG}']))
    assert sorted(a['name'] for a in final_release['assets']) == sorted(a['filename'] for a in exports)
    report = {'rewritten_commits': len(mapped), 'changed_historical_blobs': changed, 'head': mapped[observed],
              'tag': mapped[OLD_TAG], 'assets': exports, 'old_asset_names_absent': True}
    print(json.dumps(report, ensure_ascii=False, indent=2))
    summary = os.environ.get('GITHUB_STEP_SUMMARY')
    if summary:
        with open(summary, 'a') as f:
            f.write('## 개인정보 정리 결과\n\n```json\n' + json.dumps(report, ensure_ascii=False, indent=2) + '\n```\n')

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('command', choices=['directory', 'archive', 'scan', 'publish'])
    p.add_argument('paths', nargs='*')
    args = p.parse_args()
    if args.command == 'directory':
        print(json.dumps(export_directory(*args.paths), ensure_ascii=False, indent=2))
    elif args.command == 'archive':
        print(json.dumps(export_archive(*args.paths), ensure_ascii=False, indent=2))
    elif args.command == 'scan':
        scan(*args.paths)
    else:
        publish()
