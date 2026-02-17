#include <iostream>
#include <stdarg.h>
#include <time.h>
#ifndef _MSC_VER
#include <unistd.h>
#else
extern char *optarg;
extern int optind, opterr, optopt;
extern int getopt(int argc, char **argv, const char *optstring);
#endif

#include <assert.h>
#include <getopt.h>

#include "../fileio/images.h"
#include <string>
#include "CommandLineUI.h"

#include "../RayTracer.h"

using namespace std;

// The command line UI simply parses out all the arguments off
// the command line and stores them locally.
CommandLineUI::CommandLineUI(int argc, char **argv) : TraceUI() {
  int i;
  progName = argv[0];
  const char *jsonfile = nullptr;
  string cubemap_file;

  static struct option long_options[] = {
      {"aaa", no_argument, nullptr, 1000},
      {"tir", no_argument, nullptr, 1001},
      {"handle-overlap", no_argument, nullptr, 1002},
      {"fxaa", no_argument, nullptr, 1003},
      {"stochastic", no_argument, nullptr, 1004},
      {"aa-visualize", no_argument, nullptr, 1005},
      {"sinc", no_argument, nullptr, 1006},
      {"goop", no_argument, nullptr, 1007},
      {"neural", no_argument, nullptr, 1008},
      {"normal", no_argument, nullptr, 1009},
      {"parallax", no_argument, nullptr, 1010},
      {nullptr, 0, nullptr, 0}};

  while ((i = getopt_long(argc, argv, "r:w:j:c:t:n:ah", long_options,
                          nullptr)) != -1) {
    switch (i) {
    case 'r':
      m_nDepth = atoi(optarg);
      break;
    case 'w':
      m_nSize = atoi(optarg);
      break;
    case 'j':
      jsonfile = optarg;
      break;
    case 'c':
      cubemap_file = optarg;
      break;
    case 't':                   // ADDED BY ME
      m_threads = atoi(optarg); // thread count
      break;
    case 'n':
      m_nSuperSamples = atoi(optarg);
      break;
    case 'h':
      usage();
      exit(1);
    case 1000: // --aaa
      m_antialiasingAdaptive = true;
      break;
    case 1001: // --tir
      m_totalInternalReflection = true;
      break;
    case 1002: // --handle-overlap
      m_handleOverlap = true;
      break;
    case 1003: // --fxaa
      m_fxAntialiasing = true;
      break;
    case 1004: // --stochastic
      m_antialiasingStochastic = true;
      break;
    case 1005: // --aa-visualize
      m_aaVisualize = true;
      break;
    case 1006: // --sinc
      m_sincTexture = true;
      break;
    case 1007: // --goop
      m_goop = true;
      break;
    case 1008: // --neural
      m_neural = true;
      break;
    case 1009: // --normal
      m_normalMapping = true;
      break;
    case 1010: // --parallax
      m_parallaxMapping = true;
      m_normalMapping = true;
      break;
    default:
      // Oops; unknown argument
      std::cerr << "Invalid argument: '" << i << "'." << std::endl;
      usage();
      exit(1);
    }
  }
  if (jsonfile) {
    loadFromJson(jsonfile);
  }
  if (!cubemap_file.empty()) {
    smartLoadCubemap(cubemap_file);
  }

  if (optind >= argc - 1) {
    std::cerr << "no input and/or output name." << std::endl;
    exit(1);
  }

  rayName = argv[optind];
  imgName = argv[optind + 1];
}

int CommandLineUI::run() {
  assert(raytracer != 0);
  raytracer->loadScene(rayName);

  if (raytracer->sceneLoaded()) {
    int width = m_nSize;
    int height = (int)(width / raytracer->aspectRatio() + 0.5);

    raytracer->traceSetup(width, height);

    clock_t start, end;
    start = clock();

    raytracer->traceImage(width, height);
    raytracer->waitRender();
    if (aaSwitch()) {
      if (m_antialiasingStochastic) {
        raytracer->aaStochasticImage();
      } else if (m_antialiasingAdaptive) {
        raytracer->aaAdaptiveImage();
      } else if (m_fxAntialiasing) {
        raytracer->aaFastApproxImage();
      } else if (m_neural) {
        raytracer->aaNeuralImage();
      } else {
        raytracer->aaImage();
      }
      raytracer->waitRender();
    }

    end = clock();

    // save image
    unsigned char *buf;

    raytracer->getBuffer(buf, width, height);

    if (buf)
      writeImage(imgName, width, height, buf);

    if (m_aaVisualize && m_antialiasingAdaptive) {
      unsigned char *visBuf = nullptr;
      int visW = 0, visH = 0;
      raytracer->getVisualizationBuffer(visBuf, visW, visH);
      if (visBuf && visW > 0 && visH > 0) {
        std::string visName(imgName);
        size_t dot = visName.find_last_of('.');
        if (dot != std::string::npos)
          visName = visName.substr(0, dot) + "_aa_vis" + visName.substr(dot);
        else
          visName += "_aa_vis.png";
        writeImage(visName.c_str(), visW, visH, visBuf);
      }
    }

    // [[maybe_unused]]
    // double t = (double)(end - start) / CLOCKS_PER_SEC;
    // int totalRays = TraceUI::resetCount();
    // std::cout << "total time = " << t << " seconds, rays traced = " <<
    // totalRays << std::endl;

    return 0;
  } else {
    std::cerr << "Unable to load ray file '" << rayName << "'" << std::endl;
    return (1);
  }
}

void CommandLineUI::alert(const string &msg) { std::cerr << msg << std::endl; }

void CommandLineUI::usage() {
  using namespace std;
  cerr << "usage: " << progName << " [options] [input.ray output.png]" << endl
       << "  -r <#>      set recursion level (default " << m_nDepth << ")"
       << endl
       << "  -w <#>      set output image width (default " << m_nSize << ")"
       << endl
       << "  -j <FILE>   set parameters from JSON file" << endl
       << "  -c <FILE>   one Cubemap file, the remainings will be "
          "detected automatically"
       << endl;
}
