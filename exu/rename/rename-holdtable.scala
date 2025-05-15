package boom.exu

import chisel3._
import chisel3.util._
import boom.common._
import boom.util._
import freechips.rocketchip.config.Parameters

class HoldReq(val pregSz: Int) extends Bundle
{
  val prs1 = Valid(UInt(pregSz.W))
  val prs2 = Valid(UInt(pregSz.W))
  val prs3 = Valid(UInt(pregSz.W))
}

class RenameHoldTable(
  val plWidth: Int,
  val numPregs: Int,
  val bypass: Boolean,
  val float: Boolean)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    val query_resps = Output(Vec(plWidth, Bool())) // Whether the stale physical register is busy
    val query_reqs = Input(Vec(plWidth, UInt(pregSz.W))) // stale physical register

    val hold_reqs = Input(Vec(plWidth, new HoldReq(pregSz.W)))

    val early_rls_pregs = Input(Vec(plWidth, Valid(UInt(pregSz.W)))) // early release of stale physical register
    val dealloc_pregs = Input(Vec(plWidth, Valid(UInt(pregSz.W)))) // pregs returned by the ROB
    
    val recovering = Input(Bool()) // whether the pipeline is recovering from a misprediction
  })

  val hold_table = RegInit(0.U(numPregs.W))

  // 1. clear bits for early releases
  val hold_table_rls = hold_table & ~io.early_rls_pregs
    .map { rls =>
      UIntToOH(rls.bits, numPregs) & Fill(numPregs, rls.valid.asUInt)
    }.reduce(_|_)

  // 2. clear bits for deallocations
  val hold_table_dealloc = hold_table_rls & ~io.dealloc_pregs
    .map { dealloc =>
      UIntToOH(dealloc.bits, numPregs) & Fill(numPregs, dealloc.valid.asUInt)
    }.reduce(_|_)

  // 3. set bits for new hold_reqs (prs1, prs2, prs3)
  val hold_table_next = hold_table_dealloc | io.hold_reqs
    .flatMap(req => Seq(req.prs1, req.prs2, req.prs3))
    .map { v =>
      UIntToOH(v.bits, numPregs) & Fill(numPregs, v.valid.asUInt)
    }.reduce(_|_)

  

  when (!io.recovering) {
    hold_table := hold_table_next
  } .otherwise {
    // If recovering, clear the hold table
    hold_table := 0.U(numPregs.W)
  }

  // 4. query_reqs: output hold_table status for each query
  for (i <- 0 until plWidth) {
    val was_bypassed = (0 until i).map { j =>
      io.query_reqs(i) === io.hold_reqs(j).prs1.bits && io.hold_reqs(j).prs1.valid ||
      io.query_reqs(i) === io.hold_reqs(j).prs2.bits && io.hold_reqs(j).prs2.valid ||
      io.query_reqs(i) === io.hold_reqs(j).prs3.bits && io.hold_reqs(j).prs3.valid
    }.foldLeft(false.B)(_||_)

    io.query_resps(i) := hold_table(io.query_reqs(i)) || (was_bypassed && bypass.B)
  }
}