sizes=(0.1 0.316 1.0 3.16 10)

#num_threads=(384 256 192 128 96 64 32 16 8 4 2)
num_threads=(48 24 16 12 8 4 2)

rm times.txt
for size in "${sizes[@]}"
do
	for threads in "${num_threads[@]}"
	do
		echo $size $threads >> times.txt
		for cnt in {1..7}
		do
			{ time RAYON_NUM_THREADS=$threads ./target/release/rings_sim $size > out.txt ; } 2>> times.txt
		done
	done

done
