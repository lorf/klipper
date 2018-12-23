# Code for state tracking during homing operations
#
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, collections

HOMING_START_DELAY = 0.001
ENDSTOP_SAMPLE_TIME = .000015
ENDSTOP_SAMPLE_COUNT = 4

class Homing:
    def __init__(self, printer, args = {}):
        self.printer = printer
        self.toolhead = printer.lookup_object('toolhead')
        self.gcode = printer.lookup_object('gcode')
        self.changed_axes = []
        self.verify_retract = True
        self.endstops_pending = -1
        self.args = args
    def set_no_verify_retract(self):
        self.verify_retract = False
    def set_axes(self, axes):
        self.changed_axes = axes
    def get_axes(self):
        return self.changed_axes
    def _fill_coord(self, coord):
        # Fill in any None entries in 'coord' with current toolhead position
        thcoord = list(self.toolhead.get_position())
        for i in range(len(coord)):
            if coord[i] is not None:
                thcoord[i] = coord[i]
        return thcoord
    def set_homed_position(self, pos):
        self.toolhead.set_position(self._fill_coord(pos))
    def _endstop_notify(self):
        self.endstops_pending -= 1
        if not self.endstops_pending:
            self.toolhead.signal_drip_mode_end()
    def homing_move(self, movepos, endstops, speed,
                    probe_pos=False, verify_movement=False):
        # Notify endstops of upcoming home
        for mcu_endstop, name in endstops:
            mcu_endstop.home_prepare()
        # Start endstop checking
        print_time = self.toolhead.get_last_move_time()
        start_mcu_pos = [(s, name, s.get_mcu_position())
                         for es, name in endstops for s in es.get_steppers()]
        self.endstops_pending = len(endstops)
        for mcu_endstop, name in endstops:
            min_step_dist = min([s.get_step_dist()
                                 for s in mcu_endstop.get_steppers()])
            mcu_endstop.home_start(
                print_time, ENDSTOP_SAMPLE_TIME, ENDSTOP_SAMPLE_COUNT,
                min_step_dist / speed, notify=self._endstop_notify)
        self.toolhead.dwell(HOMING_START_DELAY)
        # Issue move
        error = None
        try:
            self.toolhead.drip_move(movepos, speed)
        except CommandError as e:
            error = "Error during homing move: %s" % (str(e),)
        # Wait for endstops to trigger
        move_end_print_time = self.toolhead.get_last_move_time()
        for mcu_endstop, name in endstops:
            try:
                mcu_endstop.home_wait(move_end_print_time)
            except mcu_endstop.TimeoutError as e:
                if error is None:
                    error = "Failed to home %s: %s" % (name, str(e))
        if probe_pos:
            self.set_homed_position(
                list(self.toolhead.get_kinematics().calc_position()) + [None])
        else:
            self.toolhead.set_position(movepos)
        for mcu_endstop, name in endstops:
            try:
                mcu_endstop.home_finalize()
            except CommandError as e:
                if error is None:
                    error = str(e)
        if error is not None:
            raise CommandError(error)
        # Check if some movement occurred
        if verify_movement:
            for s, name, pos in start_mcu_pos:
                if s.get_mcu_position() == pos:
                    if probe_pos:
                        raise EndstopError("Probe triggered prior to movement")
                    raise EndstopError(
                        "Endstop %s still triggered after retract" % (name,))
    def home_rails(self, rails, forcepos, movepos):
        # Alter kinematics class to think printer is at forcepos
        homing_axes = [axis for axis in range(3) if forcepos[axis] is not None]
        forcepos = self._fill_coord(forcepos)
        movepos = self._fill_coord(movepos)
        self.toolhead.set_position(forcepos, homing_axes=homing_axes)
        # Perform first home
        endstops = [es for rail in rails for es in rail.get_endstops()]
        hi = rails[0].get_homing_info()
        self.homing_move(movepos, endstops, hi.speed)
        # Perform ENDSTOP_TEST if requested
        if self.args.get('do_endstop_test'):
            assert 'endstop_test_dist' in self.args
            assert 'endstop_test_count' in self.args

            move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
            retract_r = min(1., self.args['endstop_test_dist'] / move_d)
            retractpos = [mp - ad * retract_r
                        for mp, ad in zip(movepos, axes_d)]
            forcepos = [rp - ad * retract_r
                        for rp, ad in zip(retractpos, axes_d)]
            steppers = [s for rail in rails for s in rail.get_steppers()]
            pos_ll = [[] for s in steppers];
            for n in range(self.args['endstop_test_count']):
                # Retract
                self.toolhead.move(retractpos, homing_speed)
                # Home again
                self.toolhead.set_position(forcepos)
                self.homing_move(movepos, endstops, second_homing_speed,
                                verify_movement=self.verify_retract)
                # Save current stepper positions
                spos = [s.get_mcu_position() for s in steppers]
                for pv, sp in zip(pos_ll, spos):
                    pv.append(sp)
                pos_s = " ".join(
                    ["%s:%.6fmm" % (s.get_name(), sp * s.get_step_dist())
                        for s, sp in zip(steppers, spos)])
                self.gcode.respond_info(
                        "ENDSTOP_TEST %d: %s" % (n, pos_s))
            means = [sum(v) / float(len(v)) for v in pos_ll]
            ranges = [float(max(v) - min(v)) for v in pos_ll]
            stddevs = [math.sqrt(sum([
                    (float(val) - m) ** 2 for val in v
                ]) / len(v)) for v, m in zip(pos_ll, means)]
            msg = "ENDSTOP_TEST results:\n"
            for s, p, r, sd in zip(steppers, pos_ll, ranges, stddevs):
                msg += ("%s: count:%d range:%.6fmm stddev:%.6fmm\n"
                    % (s.get_name(),
                        len(p),
                        r * s.get_step_dist(),
                        sd * s.get_step_dist()))
            self.gcode.respond_info(msg)
        # Perform second home
        elif hi.retract_dist:
            # Retract
            axes_d = [mp - fp for mp, fp in zip(movepos, forcepos)]
            move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
            retract_r = min(1., hi.retract_dist / move_d)
            retractpos = [mp - ad * retract_r
                          for mp, ad in zip(movepos, axes_d)]
            self.toolhead.move(retractpos, hi.speed)
            # Home again
            forcepos = [rp - ad * retract_r
                        for rp, ad in zip(retractpos, axes_d)]
            self.toolhead.set_position(forcepos)
            self.homing_move(movepos, endstops, hi.second_homing_speed,
                             verify_movement=self.verify_retract)
        # Signal home operation complete
        ret = self.printer.send_event("homing:homed_rails", self, rails)
        if any(ret):
            # Apply any homing offsets
            adjustpos = self.toolhead.get_kinematics().calc_position()
            for axis in homing_axes:
                movepos[axis] = adjustpos[axis]
            self.toolhead.set_position(movepos)
    def home_axes(self, axes):
        self.changed_axes = axes
        try:
            self.toolhead.get_kinematics().home(self)
        except CommandError:
            self.toolhead.motor_off()
            raise

class CommandError(Exception):
    pass

class EndstopError(CommandError):
    pass

def EndstopMoveError(pos, msg="Move out of range"):
    return EndstopError("%s: %.3f %.3f %.3f [%.3f]" % (
            msg, pos[0], pos[1], pos[2], pos[3]))

Coord = collections.namedtuple('Coord', ('x', 'y', 'z', 'e'))
