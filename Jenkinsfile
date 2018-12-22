pipeline {
  agent { dockerfile true }
  stages {
    stage('Build') {
      steps {
          sh 'mkdir build'
          sh 'cd build'
          timeout(time: 20, unit: 'MINUTES') {
            sh 'make -h'
          }
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
