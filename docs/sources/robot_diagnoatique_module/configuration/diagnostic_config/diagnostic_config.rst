Diagnostic Configuration
========================

This part of the configuration is here to define diagnostic specific options.

Legend:

- <Distance array> An array of distance in meter
- <Percentage array> An array of percentages (between 0 and 100)
- <Time array> An array of time. Allowed unit: s, ms, min
- <Ratio Array> An array of ratio (between 0 and 1). Used when values are normalized
- <Severity> An enum: ok|minor|severe|critical|~

If a value is set to [] or ~ the RDM will not flag event for this specific case. In the case of an array option, the first value is the threshold to detect a critical event, the second for a severe event and so on.
So for example, a Distance Array like this [0.001, 0.005, 0.01] will flag a minor event if the value goes under 0.01,
then reevaluates it to 0.005 if the value is inferior to 0.005
 and finally raise the severity to critical if the value goes under 0.001

::

  diagnostic_config:
    timeout_factor: <Int> Specify how much time the max signal rate has to be wait
                          to declare a topic timed out. This trigger an event.
    max_signals_delay: <Time <int> ms/s/min> Specify the maximum delay between
                                            the send time and the received time
    incident_detection: This structure specify the severity and the threshold for the event metrics
      min_event_time: <Time <int> ms/s/min> Minimal time to flag an event
      min_displayed_severity: <Enum: ok|minor|severe|critical> Minimal criticality to flag an event
      collision:
        limit_dists: <Distance array>
      computer:
        limit_cpu_loads: <Percentage array>
        limit_disk_usages: <Percentage array>
        limit_ram_usages: <Percentage array>
      clock:
        limit_utc_deviations: <Time array>
        limit_rtc_deviations: <Time array>
      node_health:
        timeout_severity: <Severity>
        not_ok_severity: <Severity>
        limit_delay_bounds: <Ratio Array>
        limit_rate_bounds: <Ratio Array>
      obstruction:
        obstruction_severity: <Severity>
      signal_health:
        nan_severity: <Severity>
        zero_severity: <Severity>
        subnormal_severity: <Severity>
        inf_severity: <Severity>
        bad_timestamp_severity: <Severity>
        limit_bounds: <Ratio Array>
      dynamic:
        limit_xdot_difference: <Ratio Array>
        input_domain_severity: <Severity>
        state_domain_severity: <Severity>
      tracking:
        limit_tracking_difference: <Ratio Array>
    upload: This configuration allow the user to reduce the bandwidth
            by not uploading high frequency metrics
      high_frequency:
        state: <Boolean>
        input: <Boolean>
        path_tracking_error: <Boolean>
        control_tracking_error: <Boolean>
