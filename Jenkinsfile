#!/usr/bin/env groovy

pipeline {
  agent none
  stages {

    stage('Build') {
      steps {
        script {
          def build_nodes = [:]
          def docker_images = [
            nuttx: "px4io/px4-dev-nuttx:2019-03-08",
          ]

          def nuttx_builds_archive = [
            target: [
                     "airmind_mindpx-v2_bootloader",
                     "av_x-v1_bootloader",
                     "bitcraze_crazyflie_bootloader",
                     "intel_aerofc-v1_bootloader",
                     "mro_x21-v1_bootloader",
                     "nxp_fmuk66-v3_bootloader",
                     "omnibus_f4sd_bootloader",
                     "px4_fmu-v2_bootloader",
                     "px4_fmu-v3_bootloader",
                     "px4_fmu-v4_bootloader",
                     "px4_fmu-v4pro_bootloader",
                     "px4_fmu-v5_bootloader",
                     "px4_io-v2_bootloader"
            ],
            image: docker_images.nuttx,
            archive: true
          ]

          def docker_builds = [
            nuttx_builds_archive
          ]

          for (def build_type = 0; build_type < docker_builds.size(); build_type++) {
            for (def build_target = 0; build_target < docker_builds[build_type].target.size(); build_target++) {
              build_nodes.put(docker_builds[build_type].target[build_target],
                createBuildNode(docker_builds[build_type].archive, docker_builds[build_type].image, docker_builds[build_type].target[build_target])
                )
            }
          }

        parallel build_nodes

        } // script
      } // steps
    } // stage Build

  } // stages
  environment {
    CCACHE_DIR = '/tmp/ccache'
    CI = true
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '10', artifactDaysToKeepStr: '28'))
    timeout(time: 60, unit: 'MINUTES')
  }
}

def createBuildNode(Boolean archive, String docker_image, String target) {
  return {

    node {
        docker.image(docker_image).inside('-e CCACHE_BASEDIR=${WORKSPACE} -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
          stage(target) {
            try {
              sh('export')
              checkout(scm)
              sh('make distclean')
              sh('git fetch --tags')
              sh('ccache -z')
              sh('make ' + target)
              sh('ccache -s')
              sh('make sizes')
              if (archive) {
                archiveArtifacts(allowEmptyArchive: false, artifacts: 'build/*/*.elf, build/*/*.bin, build/*/*.hex', fingerprint: true, onlyIfSuccessful: true)
              }
            }
            catch (exc) {
              throw (exc)
            }
            finally {
              sh('make distclean')
            }
          }
      }
    }
  }
}
