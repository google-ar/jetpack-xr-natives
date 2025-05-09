#!/usr/bin/env python

"""Spawns stubby to kick off Kokoro builds, get Kokoro status, or download tarballs.

Currently this script simply executes shell commands rather than using a Pythonic API. This is
simple and works fine for our release process.

The usual procedure is to first make a git tag (e.g. v1.0.0) and push it to GitHub. Next, do:

    $ prodaccess ; g4d myworkspace ; cd third_party/filament
    $ ./release.py kickoff v1.0.0
    $ ./release.py status
    $ ./release.py status
    $ ./release.py fetch

If you'd like to build a docs tarball that includes the latest JavaScript documentation, do the
following:

    $ cd github/filament
    $ ./build.sh -ap webgl release
    $ cd web/docs && ./build.py && cd -
    $ ~/android-tnt/build/release.py docs

Usage: python release.py [status|fetch|docs|kickoff <commitish>]'"""

import datetime
import os
import sys
import shlex
from subprocess import PIPE, Popen

PLATFORMS = 'windows web mac ios linux android'.split()
JOB_NAME = 'full_job_name: "android-tnt/PLATFORM/release"'
KICKOFF = 'scm_revision: { github_scm_revision: { commit_sha: "COMMITISH" } } ' + JOB_NAME
DATE = datetime.date.today().isoformat().replace('-', '')

ARTIFACTS = {
    'windows': [['filament-windows.tgz', 'filament-DATE-windows.tgz']],
    'web':     [['filament-release-web.tgz', 'filament-DATE-web.tgz']],
    'mac':     [['filament-release-darwin.tgz', 'filament-DATE-mac.tgz']],
    'ios':     [['filament-release-ios.tgz', 'filament-DATE-ios.tgz']],
    'linux':   [['filament-release-linux.tgz', 'filament-DATE-linux.tgz']],
    'android': [
                   ['filament-android-release.aar', 'filament-DATE-android.aar'],
                   ['filamat-android-full-release.aar', 'filamat-DATE-full-android.aar'],
                   ['filamat-android-lite-release.aar', 'filamat-DATE-lite-android.aar'],
                   ['gltfio-android-release.aar', 'gltfio-DATE-android.aar'],
               ]
}

def help():
    print(__doc__)

def kickoff(platform, commitish):
    script = KICKOFF.replace('COMMITISH', commitish)
    cmd = shlex.split('stubby call blade:kokoro-api KokoroApi.Build --batch')
    proc = Popen(cmd, stdin=PIPE, stdout=PIPE)
    result = proc.communicate(script.replace('PLATFORM', platform))
    proc.wait()
    result = result[0].split()
    buildno = result[1]
    print(platform + '\tbuild number: ' + buildno)
    return buildno

def get_status(platform):
    cmd = shlex.split('stubby call blade:kokoro-api KokoroApi.GetBuildStatus')
    proc = Popen(cmd, stdin=PIPE, stdout=PIPE)
    results = proc.communicate(JOB_NAME.replace('PLATFORM', platform))[0].split()
    proc.wait()
    status = None
    commit_sha = None
    for i in range(len(results)):
        if results[i] == 'status:':
            status = results[i + 1]
        if results[i] == 'commit_sha:' and not commit_sha:
            commit_sha = results[i + 1].strip('"')
    print(platform + '\t' + str(status) + ' ' + str(commit_sha))
    return status

def download_artifact(platform):
    cmd = shlex.split('stubby call blade:kokoro-api KokoroApi.GetBuildStatus')
    proc = Popen(cmd, stdin=PIPE, stdout=PIPE)
    results = proc.communicate(JOB_NAME.replace('PLATFORM', platform))[0].split()
    proc.wait()
    location = None
    for result in results:
        if '/build_artifacts/' in result:
            location = result
    has_location = location
    location = str(location).strip('"')
    print(platform + '\t' + location)
    if has_location:
        for pair in ARTIFACTS[platform]:
            artifact_name = pair[0]
            final_name = pair[1].replace('DATE', DATE)
            print('\tdownloading ' + artifact_name + ' => ' + final_name)
            url = location + '/' + artifact_name
            # Due to SSL, we use fileutil instead of urllib.urlretrieve
            # Also, fileutil has a cool progress indicator.
            placer_path = url[url.find('/placer'):]
            os.system('fileutil cp ' + placer_path + ' ' + final_name)

def build_docs():
    os.system('cd docs && tar --exclude=math -cvf ../filament-{0}-docs.tar *'.format(DATE)) 
    os.system('gzip -c filament-{0}-docs.tar > filament-{0}-docs.tgz'.format(DATE))
    os.system('rm filament-{0}-docs.tar'.format(DATE))

if __name__ == "__main__":
    if len(sys.argv) < 2:
        help()
        exit(1)
    command = sys.argv[1]
    if command == 'kickoff':
        if len(sys.argv) < 3:
            quit('commitish required')
        commitish = sys.argv[2]
        print('Spawning stubby, this may take a while...')
        for platform in PLATFORMS:
            kickoff(platform, commitish)
    elif command == 'status':
        for platform in PLATFORMS:
            get_status(platform)
    elif command == 'fetch':
        for platform in PLATFORMS:
            download_artifact(platform)
    elif command == 'docs':
        if not os.path.exists('.git/config'):
            quit('Must run from filament repo root.')
        repo_is_filament = open('.git/config').read().find('github.com/google/filament') > -1
        if not repo_is_filament:
            quit('Must run from filament repo root.')
        if not os.path.exists('docs/webgl'):
            quit('First, please do: cd web/docs && ./build.py')
        build_docs()
    else:
        help()
