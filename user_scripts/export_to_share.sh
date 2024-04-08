mkdir -p ~/.local/share/crow/
ln -svf -t ~/.local/share/crow/ $(readlink -e ../config/* ../lib/* ../data/*)
