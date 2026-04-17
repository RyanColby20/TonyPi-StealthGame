'''
This script will clone the repo and pull if it is already cloned
Note that *.yaml is part of .gitignore

RUN ME IN THE HOME DIRECTORY
TO BE MORE SPECIFIC: THE PARENT DIRECTORY OF TonyPi ON THE ROBOTS

'''
import os
import subprocess

REPO_URL = "https://github.com/RyanColby20/TonyPi-StealthGame.git"
BRANCH = "finalization"
TARGET_DIR = os.getcwd()
REPO_NAME = "TonyPi"

repo_path = os.path.join(TARGET_DIR, REPO_NAME)
git_dir = os.path.join(repo_path, ".git")

def get_untracked_files(repo_path):
    result = subprocess.run(
        ["git", "-C", repo_path, "ls-files", "--others", "--exclude-standard"],
        capture_output=True, text=True
    )
    return set(result.stdout.strip().split("\n")) if result.stdout else set()

def get_repo_files(repo_path, branch):
    result = subprocess.run(
        ["git", "-C", repo_path, "ls-tree", "-r", branch, "--name-only"],
        capture_output=True, text=True
    )
    return set(result.stdout.strip().split("\n")) if result.stdout else set()

def remove_conflicting_untracked(repo_path, branch):
    untracked = get_untracked_files(repo_path)
    repo_files = get_repo_files(repo_path, f"origin/{branch}")

    conflicts = untracked.intersection(repo_files)

    for f in conflicts:
        full_path = os.path.join(repo_path, f)
        if os.path.exists(full_path):
            print(f"[git-friend] Removing conflicting untracked file: {f}")
            os.remove(full_path)

    return conflicts

# Ensure TonyPi directory exists
os.makedirs(repo_path, exist_ok=True)

if not os.path.isdir(git_dir):
    print("[git-friend] Initializing new git repo in TonyPi")
    subprocess.run(["git", "-C", repo_path, "init"], check=True)
    subprocess.run(["git", "-C", repo_path, "remote", "add", "origin", REPO_URL], check=True)

print("[git-friend] Pulling latest changes....")
conflicts = remove_conflicting_untracked(repo_path, BRANCH)
subprocess.run(["git", "-C", repo_path, "pull", "origin", BRANCH], check=True)

print("[git-friend] Done.")
