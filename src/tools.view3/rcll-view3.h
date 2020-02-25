/***************************************************************************
 *  rcll-view3.h - output the communication data from ProtoBuf.
 *
 *  Created: Mon June 17 14:20:00 2013
 *  Copyright  2013  Wataru UEMURA [friede.elec.ryukoku.ac.jp]
 ****************************************************************************/

//
// refbox-view3 creates the file which includes the received data.
// 
#ifndef RCLL_VIEW3
#define RCLL_VIEW3

#include <msgs/BeaconSignal.pb.h>
#include <msgs/OrderInfo.pb.h>
#include <msgs/GameState.pb.h>
#include <msgs/VersionInfo.pb.h>
#include <msgs/ExplorationInfo.pb.h>
#include <msgs/MachineInfo.pb.h>
#include <msgs/MachineReport.pb.h>
#include <msgs/RobotInfo.pb.h>
#include <msgs/RingInfo.pb.h>

using namespace protobuf_comm;
using namespace llsf_msgs;

typedef struct
{
  std::string fileName;	// input filename
  unsigned long seq = 0;
} rcllView3Recv;

typedef struct
{
  std::string fileName;		// output filename
  std::string reportFile; 	// report file name
  std::string reportSeqFile; 	// report seq file name
  std::string prepareMachineFile;	// ofr preparing the machine
  std::string prepareMachineSeqFile;
  double x = 0;
  double y = 0;
  double ori = 0;
  unsigned long seq = 0;
  int playerNo = 1;
  unsigned long reportSeq = 0; // report sequence
  unsigned long reportOldSeq = 0;
  unsigned long prepareMachineSeq = 0;
  unsigned long prepareMachineOldSeq = 0;
  int machineRotation = 0;
  int machineZoneNumber = 0;
  int machineTagNumber = 0;
  std::string machineZoneName;
  std::string machineTpyeName;
} rcllView3Send;

rcllView3Send sendData;
rcllView3Recv recvData;

void rcllView3_init();
void rcllView3_get();
void rcllView3_put(std::shared_ptr<llsf_msgs::GameState> &gs,
                   std::shared_ptr<llsf_msgs::OrderInfo> &oi,
                   std::shared_ptr<llsf_msgs::VersionInfo> &vi,
                   std::shared_ptr<llsf_msgs::ExplorationInfo> &ei,
                   std::shared_ptr<llsf_msgs::MachineInfo> &mi,
                   std::shared_ptr<llsf_msgs::MachineReportInfo> &mri,
                   std::shared_ptr<llsf_msgs::RobotInfo> &ri,
                   std::shared_ptr<llsf_msgs::RingInfo> &rgi);
std::string convertTagToName(int tagNumber);
int convertNameToNum(std::string name);
int convertStateToNum(std::string state);
int convertTagToTeam(int tagNumber);
std::string convertNumToName(int view3Number);
std::string convertNumToBaseSide(int view3Number);
std::string convertNumToBaseColor(int view3Number);
std::string convertNumToSSOperation(int view3Number);
std::string convertNumToRingColor(int view3Number);
std::string convertNumToCSOperation(int view3Number);

#endif // RCLL_VIEW3
