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
#include "../parser/JsonParser.h"
#include "../parser/ParserException.h"
#include "../scene/scene.h"
#include <filesystem>
#include <fstream>
#include <sstream>
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
      {"animation", required_argument, nullptr, 1011},
      {"t-start", required_argument, nullptr, 1012},
      {"t-end", required_argument, nullptr, 1013},
      {"path", no_argument, nullptr, 1014},
      {"spp", required_argument, nullptr, 1015},
      {"distributed", no_argument, nullptr, 1016},
      {"caustic-photons", required_argument, nullptr, 1017},
      {"caustic-radius", required_argument, nullptr, 1018},
      {"caustic-gather", required_argument, nullptr, 1019},
      {"fresnel", no_argument, nullptr, 1020},
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
    case 1011: // --animation
      m_animationFrames = atoi(optarg);
      if (m_animationFrames < 1)
        m_animationFrames = 1;
      break;
    case 1012: // --t-start
      m_tStart = atof(optarg);
      break;
    case 1013: // --t-end
      m_tEnd = atof(optarg);
      break;
    case 1014: // --path
      m_forcePathIntegrator = true;
      break;
    case 1015: // --spp
      m_forcePathSPP = atoi(optarg);
      break;
    case 1016: // --distributed
      m_forceDistributed = true;
      break;
    case 1017: // --caustic-photons
      m_causticPhotons = atoi(optarg);
      break;
    case 1018: // --caustic-radius
      m_causticRadius = atof(optarg);
      break;
    case 1019: // --caustic-gather
      m_causticGather = atoi(optarg);
      break;
    case 1020: // --fresnel
      m_fresnel = true;
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
  if (m_forcePathIntegrator) {
    m_tracerMode = TraceUI::TracerMode::Path;
  } else if (m_forceDistributed) {
    m_tracerMode = TraceUI::TracerMode::Distributed;
  }
  if (m_forcePathSPP > 0) {
    m_pathSPP = m_forcePathSPP;
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

  if (m_animationFrames > 0) {
    return runAnimation();
  }

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

    return 0;
  } else {
    std::cerr << "Unable to load ray file '" << rayName << "'" << std::endl;
    return (1);
  }
}

int CommandLineUI::runAnimation() {
  using json = nlohmann::json;

  std::string path(rayName);
  size_t slash = path.find_last_of("\\/");
  std::string sceneDir = (slash != std::string::npos) ? path.substr(0, slash) : ".";

  std::ifstream ifs(rayName);
  if (!ifs) {
    std::cerr << "Unable to open scene file '" << rayName << "'" << std::endl;
    return 1;
  }
  std::ostringstream oss;
  oss << ifs.rdbuf();
  std::string contents = oss.str();
  ifs.close();

  json j;
  try {
    j = json::parse(contents);
  } catch (const json::exception &e) {
    std::cerr << "Invalid JSON in scene file: " << e.what() << std::endl;
    return 1;
  }

  JsonParser parser(sceneDir);
  int width = m_nSize;
  const int numFrames = m_animationFrames;
  const double tStart = m_tStart;
  const double tEnd = m_tEnd;

  {
    std::filesystem::path outPath(imgName);
    auto parent = outPath.parent_path();
    if (!parent.empty())
      std::filesystem::create_directories(parent);
  }

  for (int frame = 0; frame < numFrames; ++frame) {
    double t = (numFrames > 1)
        ? (tStart + (tEnd - tStart) * (double)frame / (numFrames - 1))
        : tStart;
    json j2 = j;
    substituteExpressions(j2, t);

    Scene *sc = nullptr;
    try {
      sc = parser.parseSceneFromJson(j2);
    } catch (const ParserException &pe) {
      std::cerr << "Frame " << frame << " (t=" << t << "): " << pe.message() << std::endl;
      return 1;
    } catch (const json::exception &je) {
      std::cerr << "Frame " << frame << " (t=" << t << "): " << je.what() << std::endl;
      return 1;
    }
    raytracer->setScene(sc);

    int height = (int)(width / raytracer->aspectRatio() + 0.5);
    raytracer->traceSetup(width, height);
    raytracer->traceImage(width, height);
    raytracer->waitRender();
    if (aaSwitch()) {
      if (m_antialiasingStochastic)
        raytracer->aaStochasticImage();
      else if (m_antialiasingAdaptive)
        raytracer->aaAdaptiveImage();
      else if (m_fxAntialiasing)
        raytracer->aaFastApproxImage();
      else if (m_neural)
        raytracer->aaNeuralImage();
      else
        raytracer->aaImage();
      raytracer->waitRender();
    }

    unsigned char *buf = nullptr;
    int outW = 0, outH = 0;
    raytracer->getBuffer(buf, outW, outH);
    if (buf && outW > 0 && outH > 0) {
      std::string frameName(imgName);
      size_t dot = frameName.find_last_of('.');
      std::string base = (dot != std::string::npos) ? frameName.substr(0, dot) : frameName;
      std::string ext = (dot != std::string::npos) ? frameName.substr(dot) : ".png";
      char fn[512];
      snprintf(fn, sizeof(fn), "%s_%04d%s", base.c_str(), frame, ext.c_str());
      try {
        writeImage(fn, outW, outH, buf);
        std::cerr << "Wrote frame " << frame << " (t=" << t << ") -> " << fn << std::endl;
      } catch (const std::string &e) {
        std::cerr << "Frame " << frame << ": failed to write " << fn << ": " << e << std::endl;
        return 1;
      }
    }
  }
  return 0;
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
          "detected automatically" << endl;
}
