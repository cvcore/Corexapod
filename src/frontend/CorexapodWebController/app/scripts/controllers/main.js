'use strict';

/**
 * @ngdoc function
 * @name corexapodControllerApp.controller:MainCtrl
 * @description
 * # MainCtrl
 * Controller of the corexapodControllerApp
 */
angular.module('corexapodControllerApp')
  .controller('MainCtrl', function($scope, $http) {
    jQuery('li.active').removeClass('active');
    jQuery('a[ng-href="#"]').parent('li').addClass('active');

    jQuery('table a').click(function() {
      var act = jQuery(this).attr('href');
      if (act == '/act/autoenable') {
        jQuery(this)
          .attr('href', '/act/autodisable')
          .html('<span class="glyphicon glyphicon-stop"></span>');
      } else if (act == '/act/autodisable') {
        jQuery(this)
          .attr('href', '/act/autoenable')
          .html('<span class="glyphicon glyphicon-play"></span>');
      } else if (act == '/act/reboot' || act == '/act/poweroff') {
        var r = confirm('确认关机或重启？');
        if (r != true)
          return false;
      }

      jQuery('table a').addClass('disabled');
      jQuery('#info')
        .addClass('alert-warning')
        .text('用户指令执行中');

      $http.get(act)
        .success(function(data) {
          jQuery('#info').removeClass('alert-warning');
          jQuery('table a').removeClass('disabled');
          jQuery('#info')
            .removeClass('alert-warning')
            .removeClass('alert-danger')
            .text('等待用户指令');
        })
        .error(function() {
          jQuery('#info')
            .removeClass('alert-warning')
            .addClass('alert-danger')
            .text('连接丢失或机器人无响应');
          jQuery('table a').removeClass('disabled');
        });
      return false;
    });

    var hadAlert = false;
    var refreshStatus = (function refreshStatus() {
      $http.get(
        '/act/status'
      ).success(function(data) {
        jQuery('#info')
          .removeClass('alert-warning')
          .removeClass('alert-danger')
          .text('等待用户指令');

        $scope.battery = data.battery;
        jQuery('.progress-bar').css('width', data.battery);
        var batteryNum = parseInt(data.battery.replace('%', ''));
        if (batteryNum <= 18) {
          if (!hadAlert) {
            alert('电量不足');
            hadAlert = true;
          }
          jQuery('#info')
            .addClass('alert-danger')
            .text('电量不足');
          jQuery('.progress-bar')
            .addClass('progress-bar-danger');
        }

        $scope.totalUseTime = data.totalUseTime;
        $scope.powerCycle = data.powerCycle;
      }).error(function() {
        jQuery('#info')
          .removeClass('alert-warning')
          .addClass('alert-danger')
          .text('连接丢失或机器人无响应');
      });
    });

    window.setInterval(refreshStatus, 1000 * 60);
    window.setTimeout(function() {
      jQuery(document.body).animate({
        scrollTop: jQuery('#info').offset().top
      }, 1000);
    }, 3000);
    refreshStatus();
  });
