# Solution because of the shared folder.
rm -r ~/mobotware/aurs-plugins/myplugin

cp -r ./myplugin ~/mobotware/aurs-plugins/myplugin

cd ~/mobotware/aurs-plugins/myplugin/
make clean && make

