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
      if ((act == '/act/reboot' || act == '/act/poweroff') && !autoOff) {
        var r = confirm('确认关机或重启？');
        if (r != true)
          return false;
      }

      jQuery('table#left a').addClass('disabled');
      jQuery('#info')
        .addClass('alert-warning')
        .text('用户指令执行中');

      $http.get(act)
        .success(function(data) {
          jQuery('#info')
            .removeClass('alert-warning')
            .removeClass('alert-danger')
            .text('等待用户指令');
          if (act == '/act/autoenable') {
            console.log(this);
            jQuery('a[href="/act/autoenable"]')
              .attr('href', '/act/autodisable')
              .html('<span class="glyphicon glyphicon-stop"></span>')
              .removeClass('disabled');
          } else if (act == '/act/poweroff') {
            clearInterval(timerRefresh);
            jQuery('#info')
              .addClass('alert-info')
              .html('<span id="countdown">30</span> 秒后可以安全关闭电源');
            var timerCountdown = window.setInterval(function() {
              var countdown = jQuery('#countdown').text();
              if (countdown == 0) {
                jQuery('#info').text('现在可以安全关闭电源');
                window.clearInterval(timerCountdown);
              }
              jQuery('#countdown').text(countdown - 1);
            }, 1000);
          } else {
            if (act == '/act/autodisable') {
              jQuery('a[href="/act/autodisable"]')
                .attr('href', '/act/autoenable')
                .html('<span class="glyphicon glyphicon-play"></span>');
            }
            jQuery('table#left a').removeClass('disabled');
          }
        })
        .error(function() {
          jQuery('#info')
            .removeClass('alert-warning')
            .addClass('alert-danger')
            .text('连接丢失或机器人无响应');
          jQuery('table#left a').removeClass('disabled');
        });
      return false;
    });

    var autoOff = false;
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
            jQuery('.progress-bar').addClass('progress-bar-danger');
            hadAlert = true;
          }
          if (batteryNum <= 8) {
            autoOff = true;
            jQuery('a[href="/act/poweroff"]').click();
          } else {
            jQuery('#info')
              .addClass('alert-danger')
              .text('电量不足');
          }
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

    var timerRefresh = window.setInterval(refreshStatus, 1000 * 60);
    refreshStatus();
  });
