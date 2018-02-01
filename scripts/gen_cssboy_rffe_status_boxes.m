function gen_cssboy_rffe_status_boxes(areas)

ncrates = size(areas,1);
nbpmslots = size(areas, 2);

wuid_counter = 0;
fid = fopen('template_rffe_status_opi_gui.xml','w');
for crate_number = 1:ncrates
    for bpmslot_number = 1:nbpmslots
        fprintf(fid, '  <!-- Generated Rectangle RFFE ON widget - Crate %d - Slot %d -->\n', crate_number, bpmslot_number);
        fprintf(fid, '  <widget typeId="org.csstudio.opibuilder.widgets.Rectangle" version="1.0.0">\n    <border_style>2</border_style>\n    <forecolor_alarm_sensitive>false</forecolor_alarm_sensitive>\n    <line_width>0</line_width>\n    <horizontal_fill>true</horizontal_fill>\n    <alarm_pulsing>false</alarm_pulsing>\n    <tooltip>$(pv_name)$(pv_value)</tooltip>\n    <rules>\n      <rule name="Connectivity" prop_id="background_color" out_exp="false">\n        <exp bool_exp="pv0 == 0">\n          <value>\n            <color red="255" green="0" blue="0" />\n          </value>\n        </exp>\n        <exp bool_exp="pv1 == 0">\n          <value>\n            <color red="255" green="0" blue="0" />\n          </value>\n        </exp>\n        <exp bool_exp="!(pv0 == 0) &amp;&amp; !(pv1 == 0)">\n          <value>\n            <color red="0" green="255" blue="0" />\n          </value>\n        </exp>\n');
        fprintf(fid, '        <pv trig="true">$(CRATE%0.2d-SLOT%0.2d_AREA)$(CRATE%0.2d-SLOT%0.2d_DEVICE)RFFEasyn.CNCT</pv>\n', crate_number, bpmslot_number, crate_number, bpmslot_number);
        fprintf(fid, '        <pv trig="true">$(CRATE%0.2d-SLOT%0.2d_AREA)$(CRATE%0.2d-SLOT%0.2d_DEVICE)RFFEasyn.ENBL</pv>\n', crate_number, bpmslot_number, crate_number, bpmslot_number);
        fprintf(fid, '      </rule>\n    </rules>\n    <enabled>true</enabled>\n');
        fprintf(fid, '  <wuid>4b176946:14ef51d6cd0:-%d</wuid>\n', 8000+wuid_counter);
        fprintf(fid, '  <transparent>false</transparent>\n    <pv_value />\n    <alpha>255</alpha>\n    <bg_gradient_color>\n      <color red="255" green="255" blue="255" />\n    </bg_gradient_color>\n    <scripts />\n    <border_alarm_sensitive>false</border_alarm_sensitive>\n    <height>20</height>\n    <border_width>2</border_width>\n    <scale_options>\n      <width_scalable>true</width_scalable>\n      <height_scalable>true</height_scalable>\n      <keep_wh_ratio>false</keep_wh_ratio>\n    </scale_options>\n    <visible>true</visible>\n    <pv_name></pv_name>\n    <gradient>false</gradient>\n    <border_color>\n      <color red="0" green="128" blue="255" />\n    </border_color>\n    <anti_alias>true</anti_alias>\n    <line_style>0</line_style>\n    <widget_type>Rectangle</widget_type>\n    <fg_gradient_color>\n      <color red="255" green="255" blue="255" />\n    </fg_gradient_color>\n    <backcolor_alarm_sensitive>true</backcolor_alarm_sensitive>\n    <background_color>\n      <color red="0" green="99" blue="255" />\n    </background_color>\n    <width>10</width>\n');
        fprintf(fid, '    <x>%d</x>\n', 190+29*(crate_number-1));
        fprintf(fid, '    <name>Rectangle_RFFE_status_%d_%d</name>\n', crate_number, bpmslot_number);
        fprintf(fid, '    <y>%d</y>\n', 82+29*(bpmslot_number-1));
        fprintf(fid, '    <fill_level>0.0</fill_level>\n    <foreground_color>\n      <color red="255" green="0" blue="0" />\n    </foreground_color>\n    <actions hook="true" hook_all="false">\n      <action type="OPEN_DISPLAY">\n        <path>BPMRFFEStatus.opi</path>\n        <macros>\n          <include_parent_macros>true</include_parent_macros>\n');
        fprintf(fid, '          <P>$(CRATE%0.2d-SLOT%0.2d_AREA)</P>\n', crate_number, bpmslot_number);
        fprintf(fid, '          <R>$(CRATE%0.2d-SLOT%0.2d_DEVICE)</R>\n', crate_number, bpmslot_number);
        fprintf(fid, '        </macros>\n        <replace>2</replace>\n        <description></description>\n      </action>\n    </actions>\n    <font>\n      <opifont.name fontName="Sans" height="10" style="0">Default</opifont.name>\n    </font>\n    <line_color>\n      <color red="128" green="0" blue="255" />\n    </line_color>\n  </widget>\n');

        wuid_counter = wuid_counter+1;
    end
end
fclose(fid);
