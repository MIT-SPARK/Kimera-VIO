/* Jenkinsfile for Jenkins running in a server using docker.
 * Run the following command to mount EUROC dataset and be able to run VIO evaluation on it:
 * sudo docker run -it -u root --rm -d -p 8080:8080 -p 50000:50000 -v /home/sparklab/Datasets/euroc:/Datasets/euroc -v \
  jenkins-data:/var/jenkins_home -v /var/run/docker.sock:/var/run/docker.sock \
  --env JAVA_OPTS="-Dhudson.model.DirectoryBrowserSupport.CSP=\"default-src 'self'; script-src * 'unsafe-eval' 'unsafe-inline'; img-src \
  'self'; style-src * 'unsafe-inline'; child-src 'self'; frame-src 'self'; object-src 'self';\"" \
  jenkinsci/blueocean
 * Periodically, you might need to clean disk space, run: `docker system prune -a` while running jenkins (but stop all jobs).
 * Also, backup: `docker cp <jenkins-container-name>:/var/jenkins_home ./jenkins_home`
 * If you want to enable HTML reports in Jenkins, further call:
 * System.setProperty("hudson.model.DirectoryBrowserSupport.CSP", "default-src 'self'; script-src * 'unsafe-eval' 'unsafe-inline'; img-src 'self'; style-src * 'unsafe-inline'; child-src 'self'; frame-src 'self'; object-src 'self';")
 * in the Script console in Jenkins' administration section.
 * TODO(Toni): change all spark_vio_evaluation/website/data into a Groovy String.
 */


pipeline {
  agent none
  stages {
    stage('Build and Test on Ubuntu') {
      parallel {
        stage('Ubuntu 18.04') {
          agent {
              dockerfile {
                filename 'Dockerfile_18_04'
                  args '-e WORKSPACE=$WORKSPACE'
              }
          }
          environment {
            evaluator="/root/spark_vio_evaluation"
          }
          stages {
            stage('Build Release') {
              steps {
               cmakeBuild buildDir: 'build', buildType: 'Release', cleanBuild: false,
                          cmakeArgs: '-DEIGEN3_INCLUDE_DIR=/usr/local/include/gtsam/3rdparty/Eigen',
                          generator: 'Unix Makefiles', installation: 'InSearchPath',
                          sourceDir: '.', steps: [[args: '-j 8']]
              }
            }
            stage('Test') {
              steps {
                wrap([$class: 'Xvfb']) {
                  sh 'cd build && ./testKimeraVIO --gtest_output="xml:testresults.xml"'
                }
              }
            }
            stage('Euroc Performance') {
              steps {
                wrap([$class: 'Xvfb']) {
                  // Copy params to Workspace
                  sh 'mkdir -p $WORKSPACE/spark_vio_evaluation/experiments'
                  sh 'cp -r $evaluator/experiments/params $WORKSPACE/spark_vio_evaluation/experiments/'

                  // Run performance tests.
                  // In jenkins_euroc.yaml, set output path to #WORKSPACE/spark_vio_evaluation/website/data
                  sh 'python3.6 $evaluator/evaluation/main_evaluation.py -r -a -v \
                    --save_plots --save_boxplots --save_results \
                    $evaluator/experiments/jenkins_euroc.yaml'

                  // Compile summary results.
                  sh 'python3.6 $evaluator/evaluation/tools/performance_summary.py \
                    spark_vio_evaluation/website/data/V1_01_easy/S/results_vio.yaml \
                    spark_vio_evaluation/website/data/V1_01_easy/S/vio_performance.csv'

                  // Copy performance website to Workspace
                  sh 'cp -r $evaluator/website $WORKSPACE/spark_vio_evaluation/'
                }
              }
              post {
                success {
                    // Plot VIO performance.
                    plot csvFileName: 'plot-vio-performance-per-build.csv',
                         csvSeries: [[file: 'spark_vio_evaluation/website/data/V1_01_easy/S/vio_performance.csv']],
                         group: 'Euroc Performance',
                         numBuilds: '30',
                         style: 'line',
                         title: 'VIO Performance',
                         yaxis: 'ATE [m]'

                    // Plot VIO timing.
                    plot csvFileName: 'plot-vio-timing-per-build.csv',
                         csvSeries: [[file: 'spark_vio_evaluation/website/data/V1_01_easy/S/output/output_timingOverall.csv']],
                         group: 'Euroc Performance',
                         numBuilds: '30',
                         style: 'line',
                         title: 'VIO Timing',
                         yaxis: 'Time [ms]'

                    // Publish HTML website with Dygraphs and pdfs of VIO performance
                    publishHTML([allowMissing: true, alwaysLinkToLastBuild: false, keepAll: true, reportDir: 'spark_vio_evaluation/website/', reportFiles: 'vio_ape_euroc.html, plots.html, datasets.html, frontend.html', reportName: 'VIO Euroc Performance Report', reportTitles: 'Euroc Performance Overview, Euroc Performance Detailed, Raw VIO Output, VIO Frontend Stats'])

                    // Archive the website
                    archiveArtifacts (
                        artifacts: 'spark_vio_evaluation/website/data/**/*.*',
                        fingerprint: true
                        )

                    // Archive the params used in evaluation (if these are used is determined
                    // by the experiments yaml file in spark_vio_evaluation)
                    archiveArtifacts (
                        artifacts: 'spark_vio_evaluation/experiments/params/**/*.*',
                        fingerprint: true
                    )
                }
                failure {
                  node(null) {
                    echo 'Fail!'
                    slackSend color: 'danger', message: "Failed - ${env.JOB_NAME} ${env.BUILD_NUMBER} (<${env.BUILD_URL}|Open>)"
                  }
                }
              }
            }
          }
        }
        stage('Ubuntu 16.04') {
          agent {
              dockerfile {
                filename 'Dockerfile_16_04'
                  args '-e WORKSPACE=$WORKSPACE'
            }
          }
          stages {
            stage('Build Release') {
              steps {
                slackSend color: 'good',
                          message: "Started Build <${env.BUILD_URL}|#${env.BUILD_NUMBER}> - Branch <${env.GIT_URL}|${env.GIT_BRANCH}>."
                            cmakeBuild buildDir: 'build', buildType: 'Release', cleanBuild: false,
                          cmakeArgs: '-DEIGEN3_INCLUDE_DIR=/usr/local/include/gtsam/3rdparty/Eigen',
                          generator: 'Unix Makefiles', installation: 'InSearchPath',
                          sourceDir: '.', steps: [[args: '-j 8']]
              }
            }
            stage('Test') {
              steps {
                wrap([$class: 'Xvfb']) {
                  sh 'cd build && ./testKimeraVIO --gtest_output="xml:testresults.xml"'
                }
              }
            }
          }
        }
      }
    }
  }
  post {
    always {
      node(null) {
        echo 'Jenkins Finished'

        // Process the CTest xml output
        junit 'build/testresults.xml'

        // Clear the source and build dirs before the next run
        deleteDir()
      }
    }
    success {
      node(null) {
        echo 'Success!'
        slackSend color: 'good',
                  message: "Successful Build <${env.BUILD_URL}|#${env.BUILD_NUMBER}> - Branch ${env.GIT_BRANCH} finished in ${currentBuild.durationString}."
      }
    }
    failure {
      node(null) {
        echo 'Fail!'
        slackSend color: 'danger', message: "Failed - ${env.JOB_NAME} ${env.BUILD_NUMBER} (<${env.BUILD_URL}|Open>)"
      }
    }
    unstable {
      node(null) {
        echo 'Unstable!'
        slackSend color: 'warning', message: "Unstable - ${env.JOB_NAME} ${env.BUILD_NUMBER} (<${env.BUILD_URL}|Open>)"
      }
    }
    cleanup {
      // Clear the source and build dirs before next run
      // TODO this might delete the .csv file for plots?
      node(null) {
        cleanWs()
      }
    }
  }
  options {
    buildDiscarder logRotator(artifactDaysToKeepStr: '120', artifactNumToKeepStr: '60', daysToKeepStr: '30', numToKeepStr: '30')
  }
}
