//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename Map Table
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._
import boom.common._
import boom.util._
import freechips.rocketchip.config.Parameters

class MapReq(val lregSz: Int) extends Bundle
{
  val lrs1 = UInt(lregSz.W)
  val lrs2 = UInt(lregSz.W)
  val lrs3 = UInt(lregSz.W)
  val ldst = UInt(lregSz.W)
}

class MapResp(val pregSz: Int) extends Bundle
{
  val prs1 = UInt(pregSz.W)
  val prs2 = UInt(pregSz.W)
  val prs3 = UInt(pregSz.W)
  val stale_pdst = UInt(pregSz.W)
}

class MapRepsAdded(val windowIdSz: Int, val robAddrSz: Int) extends Bundle
{
  val robIdx1 = UInt(robAddrSz.W) // Added
  val robIdx2 = UInt(robAddrSz.W) // Added
  val robIdx3 = UInt(robAddrSz.W) // Added
  val stale_robIdx = UInt(robAddrSz.W) // Added
  val winIdx1 = UInt(windowIdSz.W) // Added
  val winIdx2 = UInt(windowIdSz.W) // Added
  val winIdx3 = UInt(windowIdSz.W) // Added
  val stale_winIdx = UInt(windowIdSz.W) // Added
}

class RemapReq(val lregSz: Int, val pregSz: Int) extends Bundle
{
  val ldst = UInt(lregSz.W)
  val pdst = UInt(pregSz.W)
  val valid = Bool()
}

class RemapReqAdded(val windowIdSz: Int, val robAddrSz: Int) extends Bundle
{
  val winIdx = UInt(windowIdSz.W) // Added
  val robIdx = UInt(robAddrSz.W) // Added
}

class MapTableEntry(pregSz: Int, windowIdSz: Int, robAddrSz: Int) extends Bundle {
  val preg = UInt(pregSz.W)
  val winIdx = UInt(windowIdSz.W)
  val robIdx = UInt(robAddrSz.W)
}

class RenameMapTable(
  val plWidth: Int,
  val numLregs: Int,
  val numPregs: Int,
  val bypass: Boolean,
  val float: Boolean)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    // Logical sources -> physical sources.
    val map_reqs    = Input(Vec(plWidth, new MapReq(lregSz)))
    val map_resps   = Output(Vec(plWidth, new MapResp(pregSz)))
    val map_resps_added = Output(Vec(plWidth, new MapRepsAdded(pregSz, robAddrSz)))

    // Remapping an ldst to a newly allocated pdst?
    val remap_reqs  = Input(Vec(plWidth, new RemapReq(lregSz, pregSz)))
    val remap_reqs_added = Input(Vec(plWidth, new RemapReqAdded(windowIdSz, robAddrSz)))

    // Dispatching branches: need to take snapshots of table state.
    val ren_br_tags = Input(Vec(plWidth, Valid(UInt(brTagSz.W))))

    // Signals for restoring state following misspeculation.
    val brupdate      = Input(new BrUpdateInfo)
    val rollback    = Input(Bool())
  })

  // The map table register array and its branch snapshots.
  val map_table = RegInit(VecInit(Seq.fill(numLregs) {
    val entry = Wire(new MapTableEntry(pregSz, windowIdSz, robAddrSz))
    entry.preg   := 0.U
    entry.winIdx := 0.U
    entry.robIdx := 0.U
    entry
  }))
  val br_snapshots = Reg(Vec(maxBrCount, Vec(numLregs, new MapTableEntry(pregSz, windowIdSz, robAddrSz))))

  // The intermediate states of the map table following modification by each pipeline slot.
  val remap_table = Wire(Vec(plWidth+1, Vec(numLregs, new MapTableEntry(pregSz, windowIdSz, robAddrSz))))

  // Uops requesting changes to the map table.
  val remap_pdsts = io.remap_reqs map (_.pdst)
  val remap_winIdx = io.remap_reqs_added map (_.winIdx)
  val remap_robIdx = io.remap_reqs_added map (_.robIdx)
  val remap_ldsts_oh = io.remap_reqs map (req => UIntToOH(req.ldst) & Fill(numLregs, req.valid.asUInt))

  // Figure out the new mappings seen by each pipeline slot.
  for (i <- 0 until numLregs) {
    if (i == 0 && !float) { // i -> lregs
      for (j <- 0 until plWidth+1) { // j -> pipeline slots
        remap_table(j)(i) := 0.U
      }
    } else {
      val remapped_row_pdsts = (remap_ldsts_oh.map(ldst => ldst(i)) zip remap_pdsts)
        .scanLeft(map_table(i).preg) {case (pdst, (ldst, new_pdst)) => Mux(ldst, new_pdst, pdst)}

      val remapped_row_winIdx = (remap_ldsts_oh.map(ldst => ldst(i)) zip remap_winIdx)
        .scanLeft(map_table(i).winIdx) {case (winIdx, (ldst, new_winIdx)) => Mux(ldst, new_winIdx, winIdx)}

      val remapped_row_robIdx = (remap_ldsts_oh.map(ldst => ldst(i)) zip remap_robIdx)
        .scanLeft(map_table(i).robIdx) {case (robIdx, (ldst, new_robIdx)) => Mux(ldst, new_robIdx, robIdx)}
      
      for (j <- 0 until plWidth+1) {
        remap_table(j)(i).preg := remapped_row_pdsts(j)
        remap_table(j)(i).winIdx := remapped_row_winIdx(j)
        remap_table(j)(i).robIdx := remapped_row_robIdx(j)
      }
    }
  }

  // Create snapshots of new mappings.
  for (i <- 0 until plWidth) {
    when (io.ren_br_tags(i).valid) {
      br_snapshots(io.ren_br_tags(i).bits) := remap_table(i+1)
    }
  }

  when (io.brupdate.b2.mispredict) {
    // Restore the map table to a branch snapshot.
    map_table := br_snapshots(io.brupdate.b2.uop.br_tag)
  } .otherwise {
    // Update mappings.
    map_table := remap_table(plWidth)
  }

  private def query_preg(slotIdx: Int, lreg: UInt): UInt = {
    // start from the static map_table mapping
    val base = map_table(lreg).preg
    // fold through all earlier renames 0 until slotIdx
    (0 until slotIdx).foldLeft(base) { (preg, k) =>
      val req = io.remap_reqs(k)
      Mux(
        bypass.B && req.valid && req.ldst === lreg,
        req.pdst,
        preg
      )
    }
  }

  private def query_winIdx(slotIdx: Int, lreg: UInt): UInt = {
    // start from the static map_table mapping
    val base = map_table(lreg).winIdx
    // fold through all earlier renames 0 until slotIdx
    (0 until slotIdx).foldLeft(base) { (winIdx, k) =>
      val req = io.remap_reqs(k)
      Mux(
        bypass.B && req.valid && req.ldst === lreg,
        io.remap_reqs_added(k).winIdx,
        winIdx
      )
    }
  }

  private def query_robIdx(slotIdx: Int, lreg: UInt): UInt = {
    // start from the static map_table mapping
    val base = map_table(lreg).robIdx
    // fold through all earlier renames 0 until slotIdx
    (0 until slotIdx).foldLeft(base) { case (robIdx, k) =>
      val req = io.remap_reqs(k)
      Mux(
        bypass.B && req.valid && req.ldst === lreg,
        io.remap_reqs_added(k).robIdx,
        robIdx
      )
    }
  }

  // Read out mappings.
  for (i <- 0 until plWidth) {
    // basic physical‐reg lookups
    io.map_resps(i).prs1       := query_preg(i, io.map_reqs(i).lrs1)
    io.map_resps(i).prs2       := query_preg(i, io.map_reqs(i).lrs2)
    io.map_resps(i).prs3       := query_preg(i, io.map_reqs(i).lrs3)
    io.map_resps(i).stale_pdst := query_preg(i, io.map_reqs(i).ldst)

    // newly added window‐indices and ROB‐indices for each source
    io.map_resps_added(i).robIdx1 := query_robIdx(i, io.map_reqs(i).lrs1)
    io.map_resps_added(i).robIdx2 := query_robIdx(i, io.map_reqs(i).lrs2)
    io.map_resps_added(i).robIdx3 := query_robIdx(i, io.map_reqs(i).lrs3)
    io.map_resps_added(i).stale_robIdx := query_robIdx(i, io.map_reqs(i).ldst)
    io.map_resps_added(i).winIdx1 := query_winIdx(i, io.map_reqs(i).lrs1)
    io.map_resps_added(i).winIdx2 := query_winIdx(i, io.map_reqs(i).lrs2)
    io.map_resps_added(i).winIdx3 := query_winIdx(i, io.map_reqs(i).lrs3)
    io.map_resps_added(i).stale_winIdx := query_winIdx(i, io.map_reqs(i).ldst)

    if (!float) io.map_resps(i).prs3 := DontCare
    if (!float) io.map_resps_added(i).robIdx3 := DontCare
    if (!float) io.map_resps_added(i).winIdx3 := DontCare
  }

  // Don't flag the creation of duplicate 'p0' mappings during rollback.
  // These cases may occur soon after reset, as all maptable entries are initialized to 'p0'.
  io.remap_reqs map (req => (req.pdst, req.valid)) foreach {case (p,r) =>
    assert (!r || !map_table.contains(p) || p === 0.U && io.rollback, "[maptable] Trying to write a duplicate mapping.")}
}
