#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

apply_patch_once() {
  local package_path="$1"
  local patch_path="$2"

  if [[ ! -d "${repo_root}/${package_path}/.git" ]]; then
    echo "${package_path} is not a Git checkout. Run: vcs import src < dependencies.repos" >&2
    exit 1
  fi

  if git -C "${repo_root}/${package_path}" apply --check "${repo_root}/${patch_path}"; then
    git -C "${repo_root}/${package_path}" apply "${repo_root}/${patch_path}"
    echo "Applied ${patch_path}"
  elif git -C "${repo_root}/${package_path}" apply --reverse --check "${repo_root}/${patch_path}"; then
    echo "Already applied ${patch_path}"
  else
    echo "Cannot apply ${patch_path}; ${package_path} has conflicting changes." >&2
    exit 1
  fi
}

apply_patch_once "src/easy_handeye2" "third_party_patches/easy_handeye2-uam.patch"
apply_patch_once "src/ros2_aruco" "third_party_patches/ros2_aruco-uam.patch"
