

val lineRegex = """Step (\d+)|Batch: (\d+)|Common time: (.+)""".r

@main def batchInfo(fileName: String): Unit = {
	val source = scala.io.Source.fromFile(fileName)
	val lines = source.getLines()
	val pw = new java.io.PrintWriter("batches_" + fileName)
	var step = 0
	var batchSum = 0
	var batchCount = 0
	for case lineRegex(s, b, t) <- lines do {
		if s != null then {
			step = s.toInt
			if batchCount > 0 then {
				pw.println(f"$step $batchCount ${batchSum.toDouble/batchCount}")
				batchSum = 0
				batchCount = 0
			}
		} else if b != null then {
			batchCount += 1
			batchSum += b.toInt
		} else if t != null then {
		}
	}
	pw.close()
	source.close()
}
