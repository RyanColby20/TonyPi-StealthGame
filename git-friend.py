'''
This script will clone the repo and pull if it is already cloned
Note that *.yaml is part of .gitignore

'''

import os
import subprocess

REPO_URL = "https://github.com/RyanColby20/TonyPi-StealthGame.git"
TARGET_DIR = os.getcwd()          # clone/pull into current directory
REPO_NAME = "repo"                # folder name after clone

repo_path = os.path.join(TARGET_DIR, REPO_NAME)

if os.path.isdir(repo_path):
    print("Repo exists — pulling latest changes...")
    subprocess.run(["git", "-C", repo_path, "pull"], check=True)
else:
    print("Repo not found — cloning...")
    subprocess.run(["git", "clone", REPO_URL, repo_path], check=True)


