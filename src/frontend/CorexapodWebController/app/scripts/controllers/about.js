'use strict';

/**
 * @ngdoc function
 * @name corexapodControllerApp.controller:AboutCtrl
 * @description
 * # AboutCtrl
 * Controller of the corexapodControllerApp
 */
angular.module('corexapodControllerApp')
  .controller('AboutCtrl', function ($scope) {
    jQuery('li.active').removeClass('active');
    jQuery('a[ng-href="#/about"]').parent('li').addClass('active');
  });
