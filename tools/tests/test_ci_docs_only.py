"""tools/ci_docs_only.py: which pull requests skip the build and the tests."""

import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import ci_docs_only  # noqa: E402

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'ci_docs_only.py')


def test_markdown_anywhere_and_the_docs_folders_are_docs():
    assert ci_docs_only.docs_only([
        'docs/car-day.md', 'README.md', 'ros/drivers/aav_rc/CLAUDE.md',
        'docs/html/car-day.html', 'docs/html/aav-whats-new.pdf', 'docs/img/can-bus.svg',
    ])


def test_one_other_file_is_the_full_run():
    for other in ['tools/test.sh', 'pixi.lock', '.github/workflows/test.yml',
                  'docs/interfaces.yaml', 'firmware/spine/crates/ibus/src/rc.rs',
                  'docs/history/leftovers/aav-goals-steering.patch']:
        assert not ci_docs_only.docs_only(['docs/car-day.md', other]), other


def test_a_folder_named_like_docs_elsewhere_is_code():
    assert not ci_docs_only.docs_only(['ros/docs/html/x.py'])
    assert not ci_docs_only.docs_only(['docs/htmlish/x.html'])


def test_no_changed_files_is_the_full_run():
    assert not ci_docs_only.docs_only([])
    assert not ci_docs_only.docs_only([''])


def run_in(repo, base):
    return subprocess.run([sys.executable, SCRIPT, base], cwd=repo,
                          capture_output=True, text=True, check=True).stdout.strip()


def git(repo, *args):
    subprocess.run(['git', '-c', 'user.name=t', '-c', 'user.email=t@t', *args],
                   cwd=repo, check=True, capture_output=True)


def make_repo(tmp_path):
    """Make a repo whose origin is itself, so the script's fetch of the base works."""
    repo = tmp_path / 'repo'
    repo.mkdir()
    git(repo, 'init', '-q')
    git(repo, 'remote', 'add', 'origin', str(repo))
    (repo / 'tools').mkdir()
    (repo / 'tools' / 'a.py').write_text('a = 1\n')
    (repo / 'README.md').write_text('# a\n')
    git(repo, 'add', '.')
    git(repo, 'commit', '-q', '-m', 'base')
    return repo, subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=repo, check=True,
                                capture_output=True, text=True).stdout.strip()


def test_a_docs_commit_on_top_of_the_base(tmp_path):
    repo, base = make_repo(tmp_path)
    (repo / 'README.md').write_text('# a, better\n')
    git(repo, 'commit', '-q', '-am', 'docs')
    assert run_in(repo, base) == 'docs_only=true'


def test_a_py_renamed_to_md_is_code(tmp_path):
    repo, base = make_repo(tmp_path)
    git(repo, 'mv', 'tools/a.py', 'tools/a.md')
    git(repo, 'commit', '-q', '-m', 'rename')
    assert run_in(repo, base) == 'docs_only=false'


def test_a_bad_or_missing_base_is_the_full_run(tmp_path):
    repo, _ = make_repo(tmp_path)
    assert run_in(repo, '0' * 40) == 'docs_only=false'
    assert run_in(repo, '') == 'docs_only=false'
