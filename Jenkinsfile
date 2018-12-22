pipeline {
  agent { dockerfile true }
  stages {
    stage('Build') {
      steps {
          sh 'cmake .'
          timeout(time: 20, unit: 'MINUTES') {
            sh 'make -j8'
          }
      }
    }
    stage('Test') {
      steps {
        timeout(time: 30, unit: 'MINUTES') {
          sh 'make check -j8'
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
