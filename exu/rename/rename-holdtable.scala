package boom.exu

import chisel3._
import chisel3.util._
import boom.common._
import boom.util._
import freechips.rocketchip.config.Parameters

class ReHoldReq(val pregSz: Int) extends Bundle
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
    val hold_resps = Output(Vec(plWidth, Bool())) // Whether the stale physical register is busy
    val hold_reqs = Input(Vec(plWidth, UInt(pregSz.W))) // stale physical register

    val rehold_reqs = Input(Vec(plWidth, new ReHoldReq(pregSz.W)))

    val reset_pregs = Input(Vec(plWidth, Valid(UInt(pregSz.W)))) // early release of stale physical register
    val dealloc_pregs = Input(Vec(plWidth, Valid(UInt(pregSz.W)))) // pregs returned by the ROB
    
    val brupdate = Input(new BrUpdateInfo)
  })

  val hold

}