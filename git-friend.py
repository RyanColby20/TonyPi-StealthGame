'''
This script will clone the repo and pull if it is already cloned
Note that *.yaml is part of .gitignore

'''
import os
import subprocess

REPO_URL = "https://github.com/RyanColby20/TonyPi-StealthGame.git"
BRANCH = "Network_Test"
TARGET_DIR = os.getcwd()
REPO_NAME = "TonyPi"

repo_path = os.path.join(TARGET_DIR, REPO_NAME)
git_dir = os.path.join(repo_path, ".git")

# Ensure TonyPi directory exists
os.makedirs(repo_path, exist_ok=True)

if not os.path.isdir(git_dir):
    print("[git-friend] Initializing new git repo in TonyPi")
    subprocess.run(["git", "-C", repo_path, "init"], check=True)
    subprocess.run(["git", "-C", repo_path, "remote", "add", "origin", REPO_URL], check=True)

print("[git-friend] Pulling latest changes....")
subprocess.run(["git", "-C", repo_path, "pull", "origin", BRANCH], check=True)

print("[git-friend] Done.")
