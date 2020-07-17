#!/usr/bin/env groovy

pipeline {
  agent none
  stages {

    stage('Build') {
      steps {
        script {
          def build_nodes = [:]

          def docker_images = [
            nuttx: "px4io/px4-dev-nuttx:2019-02-14",
          ]

          def nuttx_builds_archive = [
            target: [
                     "auavx2v1_bl",
                     "avx_v1_bl",
                     "crazyflie_bl",
                     "cube_f4_bl",
                     "mindpxv2_bl",
                     "fmuk66v3_bl",
                     "fmuk66e_bl",
                     "omnibusf4sd_bl",
                     "pix32v5_bl",
                     "px4flow_bl",
                     "px4fmuv2_bl",
                     "px4fmuv3_bl",
                     "px4fmuv4_bl",
                     "px4fmuv4pro_bl",
                     "px4fmuv5_bl",
                     "px4io_bl",
                     "smartap_pro_bl",
                     "modalai_fc_v1_bl"
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
              sh('git clean -ff -x -d .')
              sh('git submodule update --init --recursive --force')
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
              sh('git submodule deinit -f .')
              sh('git clean -ff -x -d .')
            }
          }
      }
    }
  }
}
