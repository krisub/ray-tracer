//
// CommandLineUI.h
//
// The header file for the command line UI
//

#ifndef __CommandLineUI_h__
#define __CommandLineUI_h__

#include "TraceUI.h"

class CommandLineUI : public TraceUI {
public:
  CommandLineUI(int argc, char **argv);
  int run();

  /** trace each frame with t from m_tStart to m_tEnd */
  int runAnimation();

  void alert(const string &msg);

private:
  void usage();

  char *rayName = nullptr;
  char *imgName = nullptr;
  char *progName = nullptr;

  int m_animationFrames = 0;  // 0 = single frame
  double m_tStart = 0.0;
  double m_tEnd = 1.0;
};

#endif
