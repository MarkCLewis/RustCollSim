import collection.immutable.ArraySeq
import collection.mutable

val lines = io.Source.fromFile("event_dump.txt").getLines()
val dt = ArraySeq.fill(5)(mutable.Buffer[Double]())
val dist = mutable.Buffer[Double]()
val rel = mutable.Buffer[Double]()

for (line <- lines) {
  if (line.startsWith("dt_")) {
    val dtPart = if (line.contains(",")) line.splitAt(line.indexOf(","))(0)
      else line
    dt(dtPart(3) - 'a') += dtPart.drop(7).toDouble
  } else if (line.startsWith("dist =")) {
    val parts = line.splitAt(line.indexOf(","))
    dist += parts(0).drop(7).toDouble
    rel += parts(1).drop(8).toDouble
  }
}
