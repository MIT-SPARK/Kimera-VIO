pipeline {
  agent { dockerfile true }
  stages {
    stage('Build') {
      steps {
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
    }
    failure {
      echo 'Fail!'
    }
    unstable {
      echo 'Unstable!'
    }
  }
}
