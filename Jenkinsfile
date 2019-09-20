/* Jenkinsfile for Jenkins running in a server using docker.
 * Run the following command to mount EUROC dataset and be able to run VIO evaluation on it:
 * sudo docker run -it -u root --rm -d -p 8080:8080 -p 50000:50000 -v /home/sparklab/Datasets/euroc:/Datasets/euroc -v jenkins-data:/var/jenkins_home -v /var/run/docker.sock:/var/run/docker.sock jenkinsci/blueocean
 * If you want to enable HTML reports in Jenkins, further call:
 * System.setProperty("hudson.model.DirectoryBrowserSupport.CSP", "default-src 'self'; script-src * 'unsafe-inline'; img-src 'self'; style-src * 'unsafe-inline'; child-src 'self'; frame-src 'self'; object-src 'self';")
 * in the Script console in Jenkins' administration section.
 * TODO(Toni): change all spark_vio_evaluation/html/data into a Groovy String.
 */
pipeline {
  agent { dockerfile {
      filename 'Dockerfile'
      args '-e WORKSPACE=$WORKSPACE'
    }
  }
  stages {
    stage('Build') {
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
          sh 'cd build && ./testSparkVio --gtest_output="xml:testresults.xml"'
        }
      }
    }
    stage('Euroc Performance') {
      steps {
        wrap([$class: 'Xvfb']) {
          // Copy params to Workspace
          sh 'mkdir -p $WORKSPACE/spark_vio_evaluation/experiments'
          sh 'cp -r /root/spark_vio_evaluation/experiments/params $WORKSPACE/spark_vio_evaluation/experiments/'

          // Run performance tests.
          // In jenkins_euroc.yaml, set output path to #WORKSPACE/spark_vio_evaluation/html/data
          sh '/root/spark_vio_evaluation/evaluation/main_evaluation.py -r -a -v \
                --save_plots --save_boxplots --save_results \
                /root/spark_vio_evaluation/experiments/jenkins_euroc.yaml'

          // Compile summary results.
          sh '/root/spark_vio_evaluation/evaluation/tools/performance_summary.py \
                spark_vio_evaluation/html/data/V1_01_easy/S/results.yaml \
                spark_vio_evaluation/html/data/V1_01_easy/S/vio_performance.csv'

          // Copy performance website to Workspace
          sh 'cp -r /root/spark_vio_evaluation/html $WORKSPACE/spark_vio_evaluation/'

        }
      }
    }
  }
  post {
    always {
      echo 'Jenkins Finished'

      // Plot VIO performance.
      plot csvFileName: 'plot-vio-performance-per-build.csv',
           csvSeries: [[file: 'spark_vio_evaluation/html/data/V1_01_easy/S/vio_performance.csv']],
           group: 'Euroc Performance',
           numBuilds: '30',
           style: 'line',
           title: 'VIO Performance',
           yaxis: 'ATE [m]'

      // Plot VIO timing.
      plot csvFileName: 'plot-vio-timing-per-build.csv',
           csvSeries: [[file: 'spark_vio_evaluation/html/data/V1_01_easy/S/output/output_timingOverall.csv']],
           group: 'Euroc Performance',
           numBuilds: '30',
           style: 'line',
           title: 'VIO Timing',
           yaxis: 'Time [ms]'

      // Publish HTML website with Dygraphs and pdfs of VIO performance
      publishHTML([allowMissing: true, alwaysLinkToLastBuild: false, keepAll: true, reportDir: 'spark_vio_evaluation/html/', reportFiles: 'vio_performance.html, plots.html', reportName: 'VIO Euroc Performance Report', reportTitles: 'vio_performance, EUROC Performance'])

      // Archive the website
      archiveArtifacts (
          artifacts: 'spark_vio_evaluation/html/data/**/*.*',
          fingerprint: true
          )

      // Archive the params used in evaluation (if these are used is determined
      // by the experiments yaml file in spark_vio_evaluation)
      archiveArtifacts (
          artifacts: 'spark_vio_evaluation/experiments/params/**/*.*',
          fingerprint: true
          )

      // Process the CTest xml output
      junit 'build/testresults.xml'

      // Clear the source and build dirs before next run
      deleteDir()
    }
    success {
      echo 'Success!'
      slackSend color: 'good',
                message: "Successful Build <${env.BUILD_URL}|#${env.BUILD_NUMBER}> - Branch ${env.GIT_BRANCH} finished in ${currentBuild.durationString}."
    }
    failure {
      echo 'Fail!'
      slackSend color: 'danger', message: "Failed - ${env.JOB_NAME} ${env.BUILD_NUMBER} (<${env.BUILD_URL}|Open>)"
    }
    unstable {
      echo 'Unstable!'
      slackSend color: 'warning', message: "Unstable - ${env.JOB_NAME} ${env.BUILD_NUMBER} (<${env.BUILD_URL}|Open>)"
    }
  }
  options {
    buildDiscarder logRotator(artifactDaysToKeepStr: '120', artifactNumToKeepStr: '60', daysToKeepStr: '30', numToKeepStr: '30')
  }
}
