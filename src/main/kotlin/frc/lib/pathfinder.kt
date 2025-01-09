package frc.lib

import java.lang.Math.pow
import java.util.PriorityQueue
import kotlin.math.pow


data class Point ( val loc:Pair<Int,Int>, val from: Point?)

fun get_g(loc: Pair<Int, Int>, from: Point?, g: Double): Double {
    var currentG = g
    var currentFrom = from

    while (currentFrom?.from != null) {
        val distance = Math.sqrt(
            (loc.first - currentFrom.loc.first).toDouble().pow(2) +
                    (loc.second - currentFrom.loc.second).toDouble().pow(2)
        )
        currentG += distance
        currentFrom = currentFrom.from
    }

    if (currentFrom != null) {
        val distance = Math.sqrt(
            (loc.first - currentFrom.loc.first).toDouble().pow(2) +
                    (loc.second - currentFrom.loc.second).toDouble().pow(2)
        )
        currentG += distance
    }

    return currentG
}


fun get_f(loc: Pair<Int, Int>, dest: Pair<Int, Int>) = pow(pow(loc.first.toDouble() - dest.first.toDouble(), 2.0) + pow(loc.second.toDouble() - dest.second.toDouble(), 2.0), 0.5)

fun construct_path(closed:MutableList<Point>) : List<Pair<Int, Int>>{
    val first = closed.last()
    val list = mutableListOf(first.loc)
    var next = first.from
    while(next != null){
        list.addFirst(next.loc)
        next = next.from
    }
    return list.toList()
}

val directions = listOf(
    Pair(-1, 0), // left
    Pair(1, 0),  // right
    Pair(0, -1), // up
    Pair(0, 1),  // down
    Pair(-1, -1), // top-left diagonal
    Pair(1, -1),  // top-right diagonal
    Pair(-1, 1),  // bottom-left diagonal
    Pair(1, 1)    // bottom-right diagonal
)


fun a_star(start:Pair<Int,Int>, end:Pair<Int,Int>, grid:List<List<Boolean>>): MutableList<Point>{
    val allPoints: MutableMap<Pair<Int, Int>, Double> = mutableMapOf(
        start to (get_g(start, null, 0.0) + get_f(start, end))
    )
    val compareByLength: Comparator<Point> = compareBy { allPoints[it.loc] }
    val openqueue: PriorityQueue<Point> = PriorityQueue<Point>(compareByLength)
    val closed: MutableList<Point> = mutableListOf(Point(start, null))
    var rootNow = closed.last()
    while (rootNow.loc != end){
        for (direction in directions){
            if(rootNow.loc.second + direction.second in 0..grid.size-1){
                if(rootNow.loc.first + direction.first in 0.. grid[rootNow.loc.second + direction.second].size-1 && grid[rootNow.loc.second + direction.second][rootNow.loc.first + direction.first] && !closed.map { it.loc }.contains(Pair(rootNow.loc.first + direction.first, rootNow.loc.second + direction.second))){
                    allPoints.set(Pair(rootNow.loc.first + direction.first, rootNow.loc.second + direction.second), (get_g(Pair(rootNow.loc.first + direction.first, rootNow.loc.second + direction.second), rootNow, 0.0) + get_f(start, end)))
                    openqueue.add(Point(Pair(rootNow.loc.first + direction.first, rootNow.loc.second + direction.second), rootNow))
                }
            }
        }
        closed.addLast(openqueue.peek())
        openqueue.remove()
        rootNow = closed.last()
    }
    return closed
}

fun main() {
    val whatevs = construct_path(a_star(Pair(0,0), Pair(3,3), listOf(
        listOf(true, true, true, true),
        listOf(false, false, true, true),
        listOf(true, true, true, true),
        listOf(true, true, true, true)
    )))
    println("hi")
}
