#!/usr/bin/env bash

groupadd -g 30000 --system nixbld

for i in $(seq 1 32); do
  useradd \
    --home-dir /var/empty \
    --gid 30000 \
    --groups nixbld \
    --no-user-group \
    --system \
    --shell /usr/sbin/nologin \
    --uid $((30000 + i)) \
    --password "!" \
    nixbld$i
done

mkdir -p /root/.config/nix /root/.nixpkgs
mv /tmp/nix.conf /root/.config/nix/nix.conf
echo "{ allowUnfree = true; }" > /root/.nixpkgs/config.nix

cd /tmp || exit 1
curl -L https://nixos.org/releases/nix/nix-2.18.1/nix-2.18.1-x86_64-linux.tar.xz | tar xJf -

pushd nix-2.18.1-x86_64-linux || exit 1

mkdir -p /nix/{store,var/nix}
dest="/nix"
self=/tmp/nix-2.18.1-x86_64-linux
nix="/nix/store/azvn85cras6xv4z5j85fiy406f24r1q0-nix-2.18.1"
for i in $(cd "$self/store" >/dev/null && echo ./*); do
    if [ -t 0 ]; then
      printf "." >&2
    fi
    i_tmp="$dest/store/$i.$$"
    if [ -e "$i_tmp" ]; then
        rm -rf "$i_tmp"
    fi
    if ! [ -e "$dest/store/$i" ]; then
        cp -RPp "$self/store/$i" "$i_tmp"
        chmod -R a-w "$i_tmp"
        chmod +w "$i_tmp"
        mv "$i_tmp" "$dest/store/$i"
        chmod -w "$dest/store/$i"
    fi
done
"$nix/bin/nix-store" --load-db < "$self/.reginfo"
source "$nix/etc/profile.d/nix.sh"

popd || exit 1

/nix/store/azvn85cras6xv4z5j85fiy406f24r1q0-nix-2.18.1/bin/nix profile install
echo "export PATH=/root/.nix-profile/bin:\$PATH" >> /etc/bash.bashrc
source /etc/bash.bashrc

"$nix/bin/nix-collect-garbage" -d
"$nix/bin/nix-store" --verify --check-contents
"$nix/bin/nix" store optimise
