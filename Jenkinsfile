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
          cmakeBuild buildDir: 'build', buildType: 'Release', generator: 'Unix Makefiles', installation: 'InSearchPath', sourceDir: '.', steps: [[args: 'check -j 8']]
      }
    }
  }
  post {
    always {
      echo 'Jenkins Finished'
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
