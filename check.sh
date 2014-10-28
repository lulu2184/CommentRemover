make
res="All Correct!"
for file in test/input/*.cpp
do     
    ./remover $file test/output/${file##*/}
	if ! diff test/output/${file##*/} test/standard/${file##*/} -I test/input -w -B -E -b>nul; then
		echo "Wrong! " ${file##*/} 
		res = ""
	fi
done
echo $res
echo "DONE"
rm -rf nul *.s
make clean
