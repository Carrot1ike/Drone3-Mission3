#!/usr/bin/env bash
# Run as the normal user. Only apt/keyring/source installation uses sudo.
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$PROJECT_ROOT/logs"
exec > >(tee -a "$PROJECT_ROOT/logs/install-gazebo.log") 2>&1

source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_CODENAME:-}" != jammy ]]; then
  echo "이 스크립트는 확인된 Ubuntu 22.04 (jammy) 환경용입니다."
  exit 1
fi

echo "Gazebo Harmonic 및 CrazySim 빌드 의존성을 설치합니다."
echo "관리자 비밀번호는 터미널의 sudo 프롬프트에 입력하세요."
sudo -v

key_file="$(mktemp)"
source_file="$(mktemp)"
trap 'rm -f -- "$key_file" "$source_file"' EXIT
curl --fail --location --retry 3 --proto '=https' \
  https://packages.osrfoundation.org/gazebo.gpg --output "$key_file"
printf 'deb [arch=%s signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main\n' \
  "$(dpkg --print-architecture)" > "$source_file"
sudo install -m 0644 "$key_file" /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
sudo install -m 0644 "$source_file" /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update
sudo apt-get install --no-remove -y \
  gz-harmonic libgz-sim8-dev libgz-plugin2-dev \
  libgz-transport13-dev libgz-msgs10-dev \
  cmake build-essential pkg-config

gz sim --versions
echo "Gazebo 설치 완료. CrazySim 플러그인 빌드와 실행 검증을 이어갈 수 있습니다."
