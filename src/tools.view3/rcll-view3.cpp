
/***************************************************************************
 *  llsf-fake-robot.cpp - fake a robot
 *
 *  Created: Fri Feb 22 11:55:51 2013
 *  Copyright  2013  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the authors nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG

#include <config/yaml.h>

#include <protobuf_comm/peer.h>
#include <utils/system/argparser.h>
#include <utils/system/console_colors.h>
#include <utils/misc/string_split.h>

#include <msgs/BeaconSignal.pb.h>
#include <msgs/OrderInfo.pb.h>
#include <msgs/GameState.pb.h>
#include <msgs/VersionInfo.pb.h>
#include <msgs/ExplorationInfo.pb.h>
#include <msgs/MachineInfo.pb.h>
#include <msgs/MachineReport.pb.h>
#include <msgs/RobotInfo.pb.h>
#include <msgs/RingInfo.pb.h>

#include <boost/asio.hpp>
#include <boost/date_time.hpp>

#include "rcll-view3.h"
#include <math.h>

using namespace protobuf_comm;
using namespace llsf_msgs;
using namespace fawkes;

static boost::asio::deadline_timer *timer_ = NULL;
std::string name_;
Team team_color_;
std::string team_name_;
unsigned long seq_ = 0;
ProtobufBroadcastPeer *peer_public_ = NULL;
ProtobufBroadcastPeer *peer_team_ = NULL;
bool crypto_setup_ = false;

llsfrb::Configuration *config_;

// for prepare-machine
static bool quit = false;
std::string machine_name_;
std::string machine_type_;
llsf_msgs::MachineSide bs_side_;
llsf_msgs::BaseColor   bs_color_;
int ds_order_id_;
llsf_msgs::SSOp ss_op_;
int ss_slot_x_;
int ss_slot_y_;
int ss_slot_z_;
llsf_msgs::RingColor rs_ring_color_;
llsf_msgs::CSOp cs_operation_;


const int INTERVAL = 2500;
void
rcllView3_init()
{
	sendData.x = 0;
	sendData.y = 0;
	sendData.ori = 0;
	sendData.seq = 0;
	sendData.playerNo = 0;
	sendData.reportOldSeq = 0;

	sendData.prepareMachineSeq = 0;
	sendData.prepareMachineOldSeq = 0;

	recvData.seq = 0;
}

void
signal_handler(const boost::system::error_code& error, int signum)
{
  if (!error) {
    quit = true;

    if (timer_) {
      timer_->cancel();
    }
  }
}

void
handle_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string msg)
{
  printf("Receive error from %s:%u: %s\n",
	 endpoint.address().to_string().c_str(), endpoint.port(), msg.c_str());
}

void
handle_send_error(std::string msg)
{
  printf("Send error: %s\n", msg.c_str());
}

void
handle_message(boost::asio::ip::udp::endpoint &sender,
	       uint16_t component_id, uint16_t msg_type,
	       std::shared_ptr<google::protobuf::Message> msg)
{
  std::shared_ptr<BeaconSignal> b;
  if ((b = std::dynamic_pointer_cast<BeaconSignal>(msg))) {
#if __WORDSIZE == 64
    printf("Detected robot: %u %s:%s (seq %lu)\n",
#else
    printf("Detected robot: %u %s:%s (seq %llu)\n",
#endif
	   b->number(), b->team_name().c_str(), b->peer_name().c_str(), b->seq());
//  }
//  if ((b = std::dynamic_pointer_cast<BeaconSignal>(msg))) {
    if (b->team_name() == "LLSF" && b->peer_name() == "RefBox") {

      // read instruction file
      FILE *fp;
      int view3_machine_type_;
      int view3_bs_side_, view3_bs_color_;	// for BS
      int view3_ds_order_id_;			// for DS
      int view3_ss_op_, 
          view3_ss_slot_x_, view3_ss_slot_y_, view3_ss_slot_z_; // for SS
      int view3_rs_ring_color_;			// for RS
      int view3_cs_operation_;			// for CS
      unsigned long view3_seq;

      if ((fp = fopen(sendData.prepareMachineFile.c_str(), "r")) != NULL) {
	while(feof(fp) == 0) {
	  fscanf(fp, "%d\t", &view3_machine_type_);
	  machine_name_ = convertNumToName(view3_machine_type_);
	  machine_type_ = machine_name_.substr(2, 2);
	  if (machine_type_ == "BS") {
	    fscanf(fp, "%d\t%d\t", &view3_bs_side_, &view3_bs_color_);
	    if (! llsf_msgs::MachineSide_Parse(convertNumToBaseSide(view3_bs_side_), &bs_side_)) {
	      printf("Invalid side\n");
	    }
	    if (! llsf_msgs::BaseColor_Parse(convertNumToBaseColor(view3_bs_color_), &bs_color_)) {
	      printf("Invalid base color\n");
	    }
	  } else if (machine_type_ == "DS") {
	    fscanf(fp, "%d\t", &view3_ds_order_id_);
	    ds_order_id_ = view3_ds_order_id_;
	  } else if (machine_type_ == "SS") {
	    fscanf(fp, "%d\t%d\t%d\t%d\t", &view3_ss_op_, 
	        &view3_ss_slot_x_, &view3_ss_slot_y_, &view3_ss_slot_z_);
	    if (! llsf_msgs::SSOp_Parse(convertNumToSSOperation(view3_ss_op_), &ss_op_)) {
	      printf("Invalid operation\n");
	    }
	    ss_slot_x_ = view3_ss_slot_x_;
	    ss_slot_y_ = view3_ss_slot_y_;
	    ss_slot_z_ = view3_ss_slot_z_;
	  } else if (machine_type_ == "RS") {
	    fscanf(fp, "%d\t", &view3_rs_ring_color_);
	    if (! llsf_msgs::RingColor_Parse(convertNumToRingColor(view3_rs_ring_color_), &rs_ring_color_)) {
	      printf("Invalid ring color\n");
	    }
	  } else if (machine_type_ == "CS") {
	    fscanf(fp, "%d\t", &view3_cs_operation_);
	    if (! llsf_msgs::CSOp_Parse(convertNumToCSOperation(view3_cs_operation_), &cs_operation_)) {
	      printf("Invalid CS operation %d\n", view3_cs_operation_);
	    }
	  }
	  fscanf(fp, "%ld\t", &view3_seq);
	}
	fclose(fp);
	if (sendData.prepareMachineOldSeq != view3_seq) {
	  sendData.prepareMachineOldSeq = view3_seq;
	  sendData.prepareMachineSeq = view3_seq + 1;
	  if ((fp = fopen(sendData.prepareMachineSeqFile.c_str(), "w")) != NULL) {
	    fprintf(fp, "%ld\t", view3_seq + 1);
	    fclose(fp);
	  }
      
          printf("Announcing machine type\n");
          llsf_msgs::PrepareMachine prep;
          prep.set_team_color(team_color_);
          prep.set_machine(machine_name_);
          if (machine_type_ == "BS") {
            llsf_msgs::PrepareInstructionBS *prep_bs = prep.mutable_instruction_bs();
            prep_bs->set_side(bs_side_);
            prep_bs->set_color(bs_color_);
            printf("Set BS side %s  color %s\n", llsf_msgs::MachineSide_Name(bs_side_).c_str(),
                   llsf_msgs::BaseColor_Name(bs_color_).c_str());
          } else if (machine_type_ == "DS") {
            llsf_msgs::PrepareInstructionDS *prep_ds = prep.mutable_instruction_ds();
            prep_ds->set_order_id(ds_order_id_);
          } else if (machine_type_ == "SS") {
            llsf_msgs::PrepareInstructionSS *prep_ss = prep.mutable_instruction_ss();
/*
            llsf_msgs::SSTask *ss_task = prep_ss->mutable_task();
            ss_task->set_operation( ss_op_ );

            llsf_msgs::SSSlot *ss_slot = ss_task->mutable_shelf();
            ss_slot->set_x(ss_slot_x_);
            ss_slot->set_y(ss_slot_y_);
            ss_slot->set_z(ss_slot_z_);
*/
	    prep_ss->set_operation(ss_op_);
          } else if (machine_type_ == "RS") {
            llsf_msgs::PrepareInstructionRS *prep_rs = prep.mutable_instruction_rs();
            prep_rs->set_ring_color(rs_ring_color_);
          } else if (machine_type_ == "CS") {
            llsf_msgs::PrepareInstructionCS *prep_cs = prep.mutable_instruction_cs();
            prep_cs->set_operation(cs_operation_);
  	  }
	  peer_team_->send(prep);
	}
      }
    }
  }


  std::shared_ptr<GameState> gs;
  if ((gs = std::dynamic_pointer_cast<GameState>(msg))) {
    int hour = gs->game_time().sec() / 3600;
    int min  = (gs->game_time().sec() - hour * 3600) / 60;
    int sec  = gs->game_time().sec() - hour * 3600 - min * 60;

#if __WORDSIZE == 64
    printf("GameState received:  %02i:%02i:%02i.%02ld  %s %s  %u:%u points, %s vs. %s\n",
#else
    printf("GameState received:  %02i:%02i:%02i.%02lld  %s %s  %u:%u points, %s vs. %s\n",
#endif
	   hour, min, sec, gs->game_time().nsec() / 1000000,
	   llsf_msgs::GameState::Phase_Name(gs->phase()).c_str(),
	   llsf_msgs::GameState::State_Name(gs->state()).c_str(),
	   gs->points_cyan(), gs->points_magenta(),
	   gs->team_cyan().c_str(), gs->team_magenta().c_str());

    if (team_name_ == gs->team_cyan() || team_name_ == gs->team_magenta()) {
      if (team_name_ == gs->team_cyan() && team_color_ != CYAN) {
	printf("WARNING: sending as magenta, but our team is announced as cyan by refbox!\n");
      } else if (team_name_ == gs->team_magenta() && team_color_ != MAGENTA) {
	printf("WARNING: sending as cyan, but our team is announced as magenta by refbox!\n");
      }
      if (! crypto_setup_) {
	crypto_setup_ = true;

	std::string crypto_key = "", cipher = "aes-128-cbc";
	try {
	  crypto_key = config_->get_string(("/llsfrb/game/crypto-keys/" + team_name_).c_str());
	  printf("Set crypto key to %s (cipher %s)\n", crypto_key.c_str(), cipher.c_str());
	  peer_team_->setup_crypto(crypto_key, cipher);
	} catch (Exception &e) {
	  printf("No encryption key configured for team, not enabling crypto");
	}
      }
    } else if (crypto_setup_) {
      printf("Our team is not set, training game? Disabling crypto.\n");
      crypto_setup_ = false;
      peer_team_->setup_crypto("", "");
    }
  }

  std::shared_ptr<OrderInfo> oi;
  if ((oi = std::dynamic_pointer_cast<OrderInfo>(msg))) {
    printf("Order Info received:\n");
    for (int i = 0; i < oi->orders_size(); ++i) {
      const llsf_msgs::Order &o = oi->orders(i);
      unsigned int begin_min = o.delivery_period_begin() / 60;
      unsigned int begin_sec = o.delivery_period_begin() - begin_min * 60;
      unsigned int end_min = o.delivery_period_end() / 60;
      unsigned int end_sec = o.delivery_period_end() - end_min * 60;

      std::list<std::string> rings;
      for (int j = 0; j < o.ring_colors_size(); ++j) {
	rings.push_back(llsf_msgs::RingColor_Name(o.ring_colors(j)));;
      }

      printf("  %u (%s): %s%u%s/%s%u%s/%u of %s|%s|%s from %02u:%02u to %02u:%02u at gate %u\n", o.id(),
	     llsf_msgs::Order::Complexity_Name(o.complexity()).c_str(),
	     c_cyan, o.quantity_delivered_cyan(), c_normal,
	     c_purple, o.quantity_delivered_magenta(), c_normal,
	     o.quantity_requested(),
	     llsf_msgs::BaseColor_Name(o.base_color()).c_str(),
	     str_join(rings, "-").c_str(),
	     llsf_msgs::CapColor_Name(o.cap_color()).c_str(),
	     begin_min, begin_sec, end_min, end_sec,
	     o.delivery_gate());
    }
  }

  std::shared_ptr<VersionInfo> vi;
  if ((vi = std::dynamic_pointer_cast<VersionInfo>(msg))) {
    printf("VersionInfo received: %s\n", vi->version_string().c_str());
  }

  std::shared_ptr<ExplorationInfo> ei;
  if ((ei = std::dynamic_pointer_cast<ExplorationInfo>(msg))) {
    printf("ExplorationInfo received:\n");
    for (int i = 0; i < ei->signals_size(); ++i) {
      const ExplorationSignal &es = ei->signals(i);
      printf("  Machine type %s assignment:", es.type().c_str());
      for (int j = 0; j < es.lights_size(); ++j) {
	const LightSpec &lspec = es.lights(j);
	printf(" %s=%s", LightColor_Name(lspec.color()).c_str(),
	       LightState_Name(lspec.state()).c_str());
      }
      printf("\n");
    }
    printf("  --\n");
    for (int i = 0; i < ei->zones_size(); ++i) {
      const ExplorationZone &zm = ei->zones(i);
      printf("  Zone %s of team %s\n",
	     Zone_Name(zm.zone()).c_str(), Team_Name(zm.team_color()).c_str());
    }
  }

  std::shared_ptr<MachineInfo> mi;
  if ((mi = std::dynamic_pointer_cast<MachineInfo>(msg))) {
    printf("MachineInfo received:\n");
    for (int i = 0; i < mi->machines_size(); ++i) {
      const Machine &m = mi->machines(i);
      // const Pose2D &p = m.pose();
      const Zone &zone = m.zone();
      int rotation = m.rotation();
      float x = 1, y;
      int z = zone;
      if (z > 1000) { z -= 1000; x = -1;}
      x = ((int)(z / 10) * 1000 - 500) * x;
      y = (int)(z % 10) * 1000 - 500;
      
      printf("  %-3s|%2s|%s (%s) @ (%f, %f, %d) zone: %d\n",
	     m.name().c_str(), m.type().substr(0, 2).c_str(),
	     Team_Name(m.team_color()).substr(0, 2).c_str(),
						 m.state().c_str(),
	     x, y, rotation, z);
	           printf("  %s, prepared: %s\n", m.name().c_str(),
	     m.state() == "PREPARED" ? "YES" : "NO");
      if (0 == machine_name_.compare(m.name()) && m.state() == "PREPARED") {
        // raise (SIGINT);
        // quit = true; not quit for view3
      }
    }
  }

  std::shared_ptr<MachineReportInfo> mri;
  if ((mri = std::dynamic_pointer_cast<MachineReportInfo>(msg))) {
    printf("MachineReportInfo received:\n");
    if (mri->reported_machines_size() > 0) {
      printf("  Reported machines:");
      for (int i = 0; i < mri->reported_machines_size(); ++i) {
	printf(" %s", mri->reported_machines(i).c_str());
      }
      printf("\n");
    } else {
      printf("  no machines reported, yet\n");
    }
  }

  std::shared_ptr<RobotInfo> ri;
  if ((ri = std::dynamic_pointer_cast<RobotInfo>(msg))) {
    printf("Robot Info received:\n");
    for (int i = 0; i < ri->robots_size(); ++i) {
      const llsf_msgs::Robot &r = ri->robots(i);

      const llsf_msgs::Time &time = r.last_seen();

      boost::posix_time::ptime now(boost::posix_time::microsec_clock::universal_time());
      boost::posix_time::ptime last_seen =
	boost::posix_time::from_time_t(time.sec())
	+ boost::posix_time::nanoseconds(time.nsec());


      boost::posix_time::time_duration const last_seen_ago_td = now - last_seen;
      float last_seen_ago = last_seen_ago_td.total_milliseconds() / 1000.f;
      
      printf("  %u %s/%s @ %s: state %s, last seen %f sec ago  Maint cyc: %u  rem: %f\n",
	     r.number(), r.name().c_str(), r.team().c_str(), r.host().c_str(),
	     llsf_msgs::RobotState_Name(r.state()).substr(0,3).c_str(),
	     last_seen_ago, r.maintenance_cycles(), r.maintenance_time_remaining());
    }
  }

  std::shared_ptr<RingInfo> rgi;
  if ((rgi = std::dynamic_pointer_cast<RingInfo>(msg))) {
    printf("RingInfo received:\n");
    for (int i = 0; i < rgi->rings_size(); ++i) {
      const Ring &r = rgi->rings(i);
      printf("  %s %u\n",
						 RingColor_Name(r.ring_color()).c_str(),
						 r.raw_material());
    }
  }

  rcllView3_put(gs, oi, vi, ei, mi, mri, ri, rgi);
}


void
handle_timer(const boost::system::error_code& error)
{
  if (! error) {
    boost::posix_time::ptime now(boost::posix_time::microsec_clock::universal_time());
    std::shared_ptr<BeaconSignal> signal(new BeaconSignal());
    Time *time = signal->mutable_time();
    boost::posix_time::time_duration const since_epoch =
      now - boost::posix_time::from_time_t(0);

    time->set_sec(static_cast<google::protobuf::int64>(since_epoch.total_seconds()));
    time->set_nsec(
      static_cast<google::protobuf::int64>(since_epoch.fractional_seconds() * 
					   (1000000000/since_epoch.ticks_per_second())));
    rcllView3_get();
    if (seq_ < sendData.seq) {
      seq_ = sendData.seq;
      Pose2D *pose = signal->mutable_pose();
      pose->set_x(sendData.x);
      pose->set_y(sendData.y);
      pose->set_ori(sendData.ori);

      Time *pose_time = pose->mutable_timestamp();
      pose_time->set_sec(4);
      pose_time->set_nsec(5);

      signal->set_number(sendData.playerNo);
      signal->set_peer_name(name_);
      signal->set_team_name(team_name_);
      signal->set_team_color(team_color_);

      // signal->set_seq(++seq_);
      signal->set_seq(seq_);
      // peer_public_->send(signal);
      peer_team_->send(signal);
printf("Beacons of My Information: Position (%f, %f, %f)\n", sendData.x, sendData.y, sendData.ori);
    }

    FILE *fp;
    if ((fp = fopen(sendData.reportFile.c_str(), "r")) != NULL) {
      while(feof(fp) == 0) {
        fscanf(fp, "%d\t%d\t%d\t%ld",
          &sendData.machineRotation, &sendData.machineZoneNumber, &sendData.machineTagNumber,
          &sendData.reportSeq);
      }
      fclose(fp);
      if ((fp = fopen(sendData.reportSeqFile.c_str(), "w")) != NULL) {
        fprintf(fp, "%ld\t", sendData.reportOldSeq);
        fclose(fp);
      }
      if (sendData.reportSeq > sendData.reportOldSeq) {
        llsf_msgs::Zone machine_zone_;
        llsf_msgs::MachineReport report;
        std::string machine_name_;
        char zone_name[6]; 
        sendData.reportOldSeq = sendData.reportSeq;
        report.set_team_color(team_color_);
        llsf_msgs::MachineReportEntry *entry = report.add_machines();

	if (team_color_ != convertTagToTeam(sendData.machineTagNumber)) {
                sendData.machineZoneNumber = -sendData.machineZoneNumber;
                switch (sendData.machineRotation) {
			case   0: sendData.machineRotation = 180;
			case  45: sendData.machineRotation = 135;
			case  90: sendData.machineRotation =  90;
			case 135: sendData.machineRotation =  45;
			case 180: sendData.machineRotation =   0;
			case 225: sendData.machineRotation = 315;
			case 270: sendData.machineRotation = 270;
			case 315: sendData.machineRotation = 225;
		}
	}

        sprintf(zone_name, "%c_Z%2d", 
          sendData.machineZoneNumber < 0 ? 'M' : 'C', abs(sendData.machineZoneNumber));
        // sendData.machineZoneName = zone_name;
	// sprintf(zone_name, "M_Z12");
        if (! llsf_msgs::Zone_Parse(zone_name, &machine_zone_)) {
                  printf("Invalid zone : %s\n", zone_name);
                  sendData.machineZoneNumber = 0;
        }
        
	machine_name_ = convertTagToName(sendData.machineTagNumber - ((sendData.machineTagNumber + 1 )% 2));	// if tag is output side, tag--;
        entry->set_name(machine_name_);
        if (sendData.machineRotation >= 0) entry->set_rotation((sendData.machineRotation + (sendData.machineTagNumber % 2) * 180 + 180) % 360);
        if (sendData.machineZoneNumber != 0 ) entry->set_zone(machine_zone_);
        if (sendData.machineTagNumber != 0 && sendData.machineZoneNumber != 0) {
		peer_team_->send(report);

		// write the machine information to the file
        	FILE *fp_mi;
		std::string fileName_mi;
  		int l = recvData.fileName.length();
		float x = 1, y;
		int z = sendData.machineZoneNumber; // zone;
		if (z > 1000) { z -= 1000; x = -1;}
		x = ((int)(z / 10) * 1000 - 500) * x;
		y = (int)(z % 10) * 1000 - 500;

		fileName_mi = recvData.fileName.substr(0, l - 3) + "mI_" + machine_name_.substr(2).c_str() + ".txt";
		fp_mi = fopen(fileName_mi.c_str(), "w");

		fprintf(fp_mi, "%d\t%d\t%f\t%f\t%d\t",
			convertNameToNum(machine_name_.c_str()),
			convertStateToNum("IDLE"),
			x, y, sendData.machineRotation);
		fclose(fp_mi);
	}
      }
    }

    timer_->expires_at(timer_->expires_at()
		      + boost::posix_time::milliseconds(INTERVAL));
    timer_->async_wait(handle_timer);
  }

}

std::string  convertTagToName(int tagNumber)
{
  std::string name;
  switch (tagNumber) {
	case   1: case   2:	name = "C-CS1"; break;
	case  17: case  18:	name = "C-CS2"; break;
	case  33: case  34:	name = "C-RS1"; break;
	case 177: case 178:	name = "C-RS2"; break;
	case  65: case  66:	name = "C-BS";  break;
	case  81: case  82:	name = "C-DS";  break;
	case 193: case 194:	name = "C-SS";  break;
	case  97: case  98:	name = "M-CS1"; break;
	case 113: case 114:	name = "M-CS2"; break;
	case 129: case 130:	name = "M-RS1"; break;
	case 145: case 146:	name = "M-RS2"; break;
	case 161: case 162:	name = "M-BS";  break;
	case  49: case  50:	name = "M-DS";  break;
	case 209: case 210:	name = "M-SS";  break;
  }

  return name;
}

int convertTagToTeam(int tagNumber)
{
  switch (tagNumber) {
        case   1: case   2:
        case  17: case  18:
        case  33: case  34:
        case 177: case 178:
        case  65: case  66:
        case  81: case  82:
        case 193: case 194: return CYAN;
        case  97: case  98:
        case 113: case 114:
        case 129: case 130:
        case 145: case 146:
        case 161: case 162:
        case  49: case  50:
        case 209: case 210: return MAGENTA;
  }

  return -1;
}


int convertNameToNum(std::string name)
{
	if (name == "C-CS1") return 1;
	if (name == "C-CS2") return 2;
	if (name == "C-RS1") return 3;
	if (name == "C-RS2") return 4;
	if (name == "C-BS")  return 5;
	if (name == "C-DS")  return 6;
	if (name == "C-SS")  return 7;

	if (name == "M-CS1") return 11;
        if (name == "M-CS2") return 12;
        if (name == "M-RS1") return 13;
        if (name == "M-RS2") return 14;
        if (name == "M-BS")  return 15;
        if (name == "M-DS")  return 16;
        if (name == "M-SS")  return 17;

	return -1;
}

std::string convertNumToName(int view3Number)
{
  std::string name;
  switch (view3Number) {
        case  1:      name = "C-CS1"; break;
        case  2:      name = "C-CS2"; break;
        case  3:      name = "C-RS1"; break;
        case  4:      name = "C-RS2"; break;
        case  5:      name = "C-BS";  break;
        case  6:      name = "C-DS";  break;
        case  7:      name = "C-SS";  break;
        case 11:      name = "M-CS1"; break;
        case 12:      name = "M-CS2"; break;
        case 13:      name = "M-RS1"; break;
        case 14:      name = "M-RS2"; break;
        case 15:      name = "M-BS";  break;
        case 16:      name = "M-DS";  break;
        case 17:      name = "M-SS";  break;
  }

  return name;
}

int convertStateToNum(std::string state)
{
	if (state == "IDEL") return 1;
	if (state == "AVAILABLE") return 2;
	if (state == "PROCESSED") return 3;
	if (state == "DELIVERED") return 4;
	if (state == "RETRIEVED") return 5;
	if (state == "RESET") return 15;
	return -1;

}

std::string convertNumToBaseSide(int view3Number)
{
  std::string baseSide;
  switch (view3Number) {
        case  1:      baseSide = "INPUT"; break;
        case  2:      baseSide = "OUTPUT"; break;
  }

  return baseSide;
}

std::string convertNumToBaseColor(int view3Number)
{
  std::string baseColor;
  switch (view3Number) {
        case  1:      baseColor = "BASE_RED"; break;
        case  2:      baseColor = "BASE_BLACK"; break;
	case  3:      baseColor = "BASE_SILVER"; break;
  }

  return baseColor;
}

std::string convertNumToSSOperation(int view3Number)
{
  std::string ssOperation;
  switch (view3Number) {
        case  1:      ssOperation = "RETRIEVE"; break;
        case  2:      ssOperation = "STORE"; break;
  }

  return ssOperation;
}

std::string convertNumToRingColor(int view3Number)
{
  std::string ringColor;
  switch (view3Number) {
        case  1:      ringColor = "RING_BLUE"; break;
        case  2:      ringColor = "RING_GREEN"; break;
	case  3:      ringColor = "RING_ORANGE"; break;
	case  4:      ringColor = "RING_YELLOY"; break;
  }

  return ringColor;
}

std::string convertNumToCSOperation(int view3Number)
{
  std::string csOperation;
  switch (view3Number) {
        case  1:      csOperation = "RETRIEVE_CAP"; break;
        case  2:      csOperation = "MOUNT_CAP"; break;
  }

  return csOperation;
}


void rcllView3_get()
{
  FILE *fp;
  if ((fp = fopen(sendData.fileName.c_str(), "r")) != NULL) {
/*
    while(feof(fp) == 0) {
      fscanf(fp, "%lf\t%lf\t%lf\t%ld",
        &sendData.x, &sendData.y, &sendData.ori,
        &sendData.seq);
    }
*/
    double readData[6];
    while(feof(fp) == 0) {
      fscanf(fp, "%lf", &readData[5]);
      readData[0] = readData[1];
      readData[1] = readData[2];
      readData[2] = readData[3];
      readData[3] = readData[4];
      readData[4] = readData[5];
    }
    sendData.x   = readData[0];
    sendData.y   = readData[1];
    sendData.ori = readData[2];
    sendData.seq = (int)readData[3];

    fclose(fp);
    printf("Get data from robView: (%lf, %lf, %lf) :Seq %ld\n", sendData.x, sendData.y, sendData.ori, sendData.seq);
  }
}

void rcllView3_put(std::shared_ptr<llsf_msgs::GameState> &gs, 
                   std::shared_ptr<llsf_msgs::OrderInfo> &oi,
                   std::shared_ptr<llsf_msgs::VersionInfo> &vi,
                   std::shared_ptr<llsf_msgs::ExplorationInfo> &ei,
                   std::shared_ptr<llsf_msgs::MachineInfo> &mi,
                   std::shared_ptr<llsf_msgs::MachineReportInfo> &mri,
                   std::shared_ptr<llsf_msgs::RobotInfo> &ri,
                   std::shared_ptr<llsf_msgs::RingInfo> &rgi)
{
  FILE *fp;
  std::string fileName;
  int l = recvData.fileName.length();

  // GameState with seq number
  if (gs) {
    fileName = recvData.fileName.substr(0, l - 3) + "gameState.txt";
    if ((fp = fopen(fileName.c_str(), "w")) != NULL) {
      fprintf(fp, "%ld\t%ld\t%d\t%d\t%d\t%d\t", 
        seq_, gs->game_time().sec(), gs->phase(), gs->state(), gs->points_cyan(), gs->points_magenta());
      fclose(fp);
    }
  }

  // OrderInfo
  if (oi) {
    fileName = recvData.fileName.substr(0, l - 3) + "orderInfo.txt";
    if ((fp = fopen(fileName.c_str(), "w")) != NULL) {
      fprintf(fp, "%d\t", oi->orders_size());

      for (int i = 0; i < oi->orders_size(); ++i) {
        const llsf_msgs::Order &o = oi->orders(i);
        std::list<std::string> rings;
        fprintf(fp, "%d\t", o.ring_colors_size());
        for (int j = 0; j < o.ring_colors_size(); ++j) {
          fprintf(fp, "%d\t", o.ring_colors(j));;
        }

        fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t",
                    o.id(), o.complexity(), o.quantity_delivered_cyan(), o.quantity_delivered_magenta(),
                    o.quantity_requested(), o.base_color(), o.cap_color(), o.delivery_period_begin(),
                    o.delivery_period_end(), o.delivery_gate());
      }
      fclose(fp);
    }
  }

  // VersionInfo
  if (vi) {
    fileName = recvData.fileName.substr(0, l - 3) + "versionInfo.txt";
    if ((fp = fopen(fileName.c_str(), "w")) != NULL) {
      fprintf(fp, "%s\t",
		vi->version_string().c_str());
      fclose(fp);
    }
  }

  // ExplorationInfo
  if (ei) {
    fileName = recvData.fileName.substr(0, l - 3) + "explorationInfo.txt";
    if ((fp = fopen(fileName.c_str(), "w")) != NULL) {
      fprintf(fp, "%d\t", ei->signals_size());
      for (int i = 0; i < ei->signals_size(); ++i) {
        const ExplorationSignal &es = ei->signals(i);
        fprintf(fp, "%s\t%d\t", es.type().c_str(), es.lights_size());
        for (int j = 0; j < es.lights_size(); ++j) {
          const LightSpec &lspec = es.lights(j);
          fprintf(fp, "%d\t%d\t", lspec.color(), lspec.state());
        }
      }
      fprintf(fp, "%d\t", ei->zones_size());
      for (int i = 0; i < ei->zones_size(); ++i) {
        const ExplorationZone &zm = ei->zones(i);
        fprintf(fp, "%d\t%d\t", zm.zone(), zm.team_color());
      }
      fclose(fp);
    }
  }

  // MachineInfo
  if (mi) {
    fileName = recvData.fileName.substr(0, l - 3) + "machineInfo.txt";
    if ((fp = fopen(fileName.c_str(), "w")) != NULL) {
      fprintf(fp, "%d\t", mi->machines_size());
      for (int i = 0; i < mi->machines_size(); ++i) {
        const Machine &m = mi->machines(i);
        // const Pose2D &p = m.pose();
        const Zone &zone = m.zone();
        int rotation = m.rotation();
        float x = 1, y;
        int z = zone;
        if (z > 1000) { z -= 1000; x = -1;}
        x = ((int)(z / 10) * 1000 - 500) * x;
        y = (int)(z % 10) * 1000 - 500;


/*        fprintf(fp, "%-3s\t%2s\t%d\t%s\t%f\t%f\t%f\t",
             m.name().c_str(), m.type().substr(0, 2).c_str(),
             m.team_color(), m.state().c_str(),
             p.x(), p.y(), p.ori());
*/
        fprintf(fp, "%d\t%d\t%f\t%f\t%d\t",
                convertNameToNum(m.name().c_str()),
                convertStateToNum(m.state().c_str()),
                x, y, rotation);

	// make each machine file
	FILE *fp_mi;
	std::string fileName_mi;
	float front_x, front_y;
	fileName_mi = recvData.fileName.substr(0, l - 3) + "mI_" + m.name().substr(2).c_str() + ".txt";
	fp_mi = fopen(fileName_mi.c_str(), "w");
	front_x = cos(rotation / 360.0 * 2.0 * 3.14159) * 1000.0;
	front_y = sin(rotation / 360.0 * 2.0 * 3.14159) * 1000.0;
	        
	fprintf(fp_mi, "%d\t%d\t%f\t%f\t%d\t",
                convertNameToNum(m.name().c_str()),
                convertStateToNum(m.state().c_str()),
                x, y, rotation);
	fprintf(fp_mi, "%f\t%f\t%f\t%f\t", x + front_x, y + front_y,
					   x - front_x, y - front_y);
	fclose(fp_mi);
      }
      fclose(fp);
    }
  }

  // MachineReportInfo
  if (mri) {
    fileName = recvData.fileName.substr(0, l - 3) + "machineReportInfo.txt";
    if ((fp = fopen(fileName.c_str(), "w")) != NULL) {
      fprintf(fp, "%d\t", mri->reported_machines_size());
      if (mri->reported_machines_size() > 0) {
        for (int i = 0; i < mri->reported_machines_size(); ++i) {
          fprintf(fp, "%s\t", mri->reported_machines(i).c_str());
        }
      }
      fclose(fp);
    }
  }

  // RobotInfo
  // ****************

  // RingInfo
  if (rgi) {
    fileName = recvData.fileName.substr(0, l - 3) + "ringInfo.txt";
    if ((fp = fopen(fileName.c_str(), "w")) != NULL) {
      fprintf(fp, "%d\t", rgi->rings_size());
      for (int i = 0; i < rgi->rings_size(); ++i) {
        const Ring &r = rgi->rings(i);
        fprintf(fp, "%d\t%u\t", r.ring_color(), r.raw_material());
      }
      fclose(fp);
    }
  }

}

int
main(int argc, char **argv)
{
  rcllView3_init();

/*
  ArgumentParser argp(argc, argv, "T:");

  if (argp.num_items() != 2) {
    printf("Usage: %s <name> <team>\n", argv[0]);
    exit(1);
  }

  name_ = argp.items()[0];
  team_name_ = argp.items()[1];
*/

  team_color_ = CYAN;
/*
  if (argp.has_arg("T")) {
    std::string team_str = argp.arg("T");
    if (team_str == "cyan") {
      team_color_ = CYAN;
    } else if (team_str == "magenta") {
      team_color_ = MAGENTA;
    } else {
      printf("Unknown team value, using cyan\n");
    }
  }
*/

  config_ = new llsfrb::YamlConfiguration(CONFDIR);
  config_->load("config.yaml");

  std::string team_str;
  name_ = config_->get_string("/rcllView3/robot/name");
  team_name_ = config_->get_string("/rcllView3/team/name");
  sendData.playerNo = config_->get_uint("/rcllView3/robot/playerno");
  team_str = config_->get_string("/rcllView3/team/color");
  if (team_str == "cyan") {
    team_color_ = CYAN;
  } else if (team_str == "magenta") {
    team_color_ = MAGENTA;
  } else {
    printf("Unknown team value, using cyan\n");
  }
  sendData.fileName = config_->get_string("/rcllView3/team/sendFileName");
  sendData.reportFile = config_->get_string("/rcllView3/team/reportFileName");
  sendData.reportSeqFile = config_->get_string("/rcllView3/team/reportSeqFileName");
  sendData.prepareMachineFile = config_->get_string("/rcllView3/team/prepareMachineFileName");
  sendData.prepareMachineSeqFile =config_->get_string("/rcllView3/team/prepareMachineSeqFileName");

  recvData.fileName = config_->get_string("/rcllView3/team/recvFileName");


  if (config_->exists("/llsfrb/comm/public-peer/send-port") &&
      config_->exists("/llsfrb/comm/public-peer/recv-port") )
  {
    peer_public_ = new ProtobufBroadcastPeer(config_->get_string("/llsfrb/comm/public-peer/host"),
					     config_->get_uint("/llsfrb/comm/public-peer/recv-port"),
					     config_->get_uint("/llsfrb/comm/public-peer/send-port"));
  } else {
    peer_public_ = new ProtobufBroadcastPeer(config_->get_string("/llsfrb/comm/public-peer/host"),
					     config_->get_uint("/llsfrb/comm/public-peer/port"));
  }

  MessageRegister & message_register = peer_public_->message_register();
  message_register.add_message_type<BeaconSignal>();
  message_register.add_message_type<OrderInfo>();
  message_register.add_message_type<GameState>();
  message_register.add_message_type<VersionInfo>();
  message_register.add_message_type<ExplorationInfo>();
  message_register.add_message_type<MachineInfo>();
  message_register.add_message_type<MachineReportInfo>();
  message_register.add_message_type<RobotInfo>();
  message_register.add_message_type<RingInfo>();

  std::string cfg_prefix =
    std::string("/llsfrb/comm/") +
    ((team_color_ == CYAN) ? "cyan" : "magenta") + "-peer/";

  /*
  // better to this dynamically be reacting to the public GameState
  // this way you can also play unencrypted training games
  std::string crypto_key = "", cipher = "aes-128-cbc";
  try {
    crypto_key = config_->get_string(("/llsfrb/game/crypto-keys/" + team_name_).c_str());
  } catch (Exception &e) {
    printf("No encryption key configured for team, not enabling crypto");
  }
  */

  if (config_->exists((cfg_prefix + "send-port").c_str()) &&
      config_->exists((cfg_prefix + "recv-port").c_str()) )
  {
    peer_team_ = new ProtobufBroadcastPeer(config_->get_string((cfg_prefix + "host").c_str()),
					   config_->get_uint((cfg_prefix + "recv-port").c_str()),
					   config_->get_uint((cfg_prefix + "send-port").c_str()),
					   &message_register /*, crypto_key, cipher*/);
  } else {
    peer_team_ = new ProtobufBroadcastPeer(config_->get_string((cfg_prefix + "host").c_str()),
					   config_->get_uint((cfg_prefix + "port").c_str()),
					   &message_register /*, crypto_key, cipher*/);
  }

  boost::asio::io_service io_service;

  peer_public_->signal_received().connect(handle_message);
  peer_public_->signal_recv_error().connect(handle_recv_error);
  peer_public_->signal_send_error().connect(handle_send_error);

  peer_team_->signal_received().connect(handle_message);
  peer_team_->signal_recv_error().connect(handle_recv_error);
  peer_team_->signal_send_error().connect(handle_send_error);

#if BOOST_ASIO_VERSION >= 100601
  // Construct a signal set registered for process termination.
  boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);

  // Start an asynchronous wait for one of the signals to occur.
  signals.async_wait(signal_handler);
#endif

  timer_ = new boost::asio::deadline_timer(io_service);
  timer_->expires_from_now(boost::posix_time::milliseconds(INTERVAL));
  timer_->async_wait(handle_timer);

  do {
    io_service.run();
    io_service.reset();
  } while (! quit);

  delete timer_;
  delete peer_team_;
  delete peer_public_;
  delete config_;

  // Delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();
}
