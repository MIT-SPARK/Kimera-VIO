pipeline {
  agent { dockerfile true }
  stages {
    stage('Build') {
      steps {
          slackSend color: 'good', message: "Started Build <${env.BUILD_URL}|#${env.BUILD_NUMBER}> - Branch <${env.GIT_URL}/tree/${env.GIT_BRANCH}|${env.GIT_BRANCH}> (${GIT_COMMIT:1:10}) (author: ${env.GIT_AUTHOR_NAME})."
          cmakeBuild buildDir: 'build', buildType: 'Release', cleanBuild: true, generator: 'Unix Makefiles', installation: 'InSearchPath', sourceDir: '.', steps: [[args: '-j 8']]
        }
      }
    stage('Test') {
      steps {
          cmakeBuild buildDir: 'build', buildType: 'Release', cleanBuild: true, generator: 'Unix Makefiles', installation: 'InSearchPath', sourceDir: '.', steps: [[args: 'check -j 8']]
          ctest arguments: '-T test --no-compress-output --verbose', installation: 'InSearchPath', workingDir: 'build/tests'
      }
    }
  }
  post {
    always {
      echo 'Jenkins Finished'
      // Archive the CTest xml output
      archiveArtifacts (
          artifacts: 'build/tests/Testing/**/*.xml',
          fingerprint: true
          )

      // Process the CTest xml output with the xUnit plugin
      xunit([CTest(
            deleteOutputFiles: true,
            failIfNotNew: true,
            pattern: 'build/tests/Testing/**/*.xml',
            skipNoTestFiles: false,
            stopProcessingIfError: true)
      ])

      // Clear the source and build dirs before next run
      deleteDir()
    }
    success {
      echo 'Success!'
      slackSend color: 'good', message: "Successful Build <${env.BUILD_URL}|#${env.BUILD_NUMBER}> - Branch ${env.GIT_BRANCH} (${GIT_COMMIT}) finished in ${currentBuild.durationString} (author: ${env.GIT_AUTHOR_NAME})."
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
}
