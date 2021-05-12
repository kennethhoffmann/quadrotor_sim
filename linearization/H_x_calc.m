function H_x = H_x_calc(q_w,q_x,q_y,q_z,u1,u2,u3,u4,v_x,v_y,v_z)
%H_X_CALC
%    H_X = H_X_CALC(Q_W,Q_X,Q_Y,Q_Z,U1,U2,U3,U4,V_X,V_Y,V_Z)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    23-Feb-2021 13:11:28

t2 = q_w.*2.0;
t3 = q_x.*2.0;
t4 = q_y.*2.0;
t5 = q_z.*2.0;
t6 = q_w.^2;
t7 = q_x.^2;
t8 = q_y.^2;
t9 = q_z.^2;
t10 = u1.^2;
t11 = u2.^2;
t12 = u3.^2;
t13 = u4.^2;
t14 = v_x.*2.0;
t15 = v_y.*2.0;
t25 = q_w.*q_y.*-2.0;
t26 = q_x.*q_y.*-2.0;
t48 = q_w./7.7e+5;
t49 = q_x./7.7e+5;
t50 = q_y./7.7e+5;
t51 = q_z./7.7e+5;
t52 = q_w./1.255e+6;
t53 = q_x./1.255e+6;
t54 = q_y./1.255e+6;
t55 = q_z./1.255e+6;
t56 = v_x./7.7e+5;
t57 = v_y./7.7e+5;
t58 = v_z./7.7e+5;
t59 = v_x./1.255e+6;
t60 = v_y./1.255e+6;
t61 = v_z./1.255e+6;
t79 = q_w./3.25e+8;
t80 = q_x./3.25e+8;
t81 = q_y./3.25e+8;
t82 = q_z./3.25e+8;
t83 = v_x./3.25e+8;
t84 = v_y./3.25e+8;
t85 = v_z./3.25e+8;
t16 = q_x.*t2;
t17 = q_y.*t2;
t18 = q_z.*t2;
t19 = q_y.*t3;
t20 = q_z.*t3;
t21 = q_z.*t4;
t22 = -t4;
t23 = -t5;
t24 = -t15;
t27 = -t6;
t28 = -t7;
t29 = -t9;
t30 = t2+t5;
t31 = t3+t4;
t32 = t14+t15;
t63 = -t48;
t64 = -t49;
t65 = -t50;
t66 = -t51;
t67 = -t52;
t68 = -t53;
t69 = -t54;
t70 = -t55;
t71 = -t56;
t72 = -t57;
t73 = -t58;
t74 = -t59;
t75 = -t60;
t76 = -t61;
t86 = -t79;
t87 = -t80;
t88 = -t81;
t89 = -t82;
t90 = -t83;
t91 = -t84;
t92 = -t85;
t111 = t10.*2.7224e-8;
t112 = t11.*2.7224e-8;
t113 = t12.*2.7224e-8;
t114 = t13.*2.7224e-8;
t33 = t2+t23;
t34 = t3+t22;
t35 = t14+t24;
t36 = t30.*v_x;
t37 = t31.*v_x;
t38 = t30.*v_y;
t39 = t31.*v_y;
t40 = t30.*v_z;
t41 = t31.*v_z;
t93 = t16+t20+t21+t25;
t96 = t6+t8+t18+t19+t28+t29;
t97 = t8+t9+t18+t26+t27+t28;
t100 = (t6+t7-t8-t18+t19+t29).^2;
t101 = -v_x.*(t6+t7-t8-t18+t19+t29);
t106 = v_x.*(t6+t7-t8-t18+t19+t29);
t115 = -t111;
t116 = -t112;
t117 = -t113;
t118 = -t114;
t42 = t33.*v_x;
t43 = t34.*v_x;
t44 = t33.*v_y;
t45 = t34.*v_y;
t46 = t33.*v_z;
t47 = t34.*v_z;
t62 = -t36;
t94 = t93.*v_z;
t95 = t93.^2;
t98 = t96.^2;
t99 = t96.*v_y;
t121 = q_w.*q_x.*t93.*t96.*4.0e-9;
t122 = q_w.*q_y.*t93.*t96.*4.0e-9;
t123 = q_x.*q_z.*t93.*t96.*4.0e-9;
t124 = q_y.*q_z.*t93.*t96.*4.0e-9;
t125 = t6.*t93.*t96.*2.0e-9;
t126 = t7.*t93.*t96.*2.0e-9;
t127 = t8.*t93.*t96.*2.0e-9;
t128 = t9.*t93.*t96.*2.0e-9;
t129 = q_w.*q_x.*t93.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t131 = q_w.*q_y.*t93.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t132 = q_x.*q_z.*t93.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t133 = q_y.*q_z.*t93.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t134 = t6.*t93.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t135 = t7.*t93.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t137 = t8.*t93.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t139 = t9.*t93.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t140 = q_w.*q_y.*t93.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t141 = q_x.*q_z.*t93.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t142 = q_y.*q_z.*t93.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t143 = t6.*t93.*(t6+t7-t8-t18+t19+t29).*2.0e-9;
t144 = t9.*t93.*(t6+t7-t8-t18+t19+t29).*2.0e-9;
t189 = q_w.*q_x.*t96.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t190 = q_w.*q_y.*t96.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t191 = q_x.*q_z.*t96.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t192 = q_y.*q_z.*t96.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t193 = t6.*t96.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t194 = t7.*t96.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t195 = t8.*t96.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t196 = t9.*t96.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t197 = q_w.*q_y.*t96.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t198 = q_x.*q_z.*t96.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t199 = q_y.*q_z.*t96.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t200 = t6.*t96.*(t6+t7-t8-t18+t19+t29).*2.0e-9;
t201 = t9.*t96.*(t6+t7-t8-t18+t19+t29).*2.0e-9;
t77 = -t45;
t78 = -t46;
t102 = t38+t42+t47;
t104 = t41+t44+t62;
t119 = t94+t99+t106;
t130 = -t121;
t136 = -t126;
t138 = -t127;
t513 = t122+t123;
t515 = t129+t142;
t516 = t140+t141;
t517 = t189+t199;
t518 = t197+t198;
t520 = t135+t137+t143+t144;
t521 = t194+t195+t200+t201;
t103 = t37+t40+t77;
t105 = t102.^2;
t107 = t39+t43+t78;
t109 = t104.^2;
t120 = t119.^2;
t145 = q_w.*q_x.*t93.*t102.*4.0e-9;
t146 = q_w.*q_y.*t93.*t102.*4.0e-9;
t147 = q_x.*q_z.*t93.*t102.*4.0e-9;
t148 = q_y.*q_z.*t93.*t102.*4.0e-9;
t149 = t6.*t93.*t102.*2.0e-9;
t150 = t7.*t93.*t102.*2.0e-9;
t151 = t8.*t93.*t102.*2.0e-9;
t152 = t9.*t93.*t102.*2.0e-9;
t155 = q_w.*q_x.*t93.*t104.*4.0e-9;
t157 = q_w.*q_y.*t93.*t104.*4.0e-9;
t159 = q_x.*q_z.*t93.*t104.*4.0e-9;
t161 = q_y.*q_z.*t93.*t104.*4.0e-9;
t163 = t6.*t93.*t104.*2.0e-9;
t166 = t7.*t93.*t104.*2.0e-9;
t169 = t8.*t93.*t104.*2.0e-9;
t171 = t9.*t93.*t104.*2.0e-9;
t202 = q_w.*q_x.*t96.*t102.*4.0e-9;
t203 = q_w.*q_y.*t96.*t102.*4.0e-9;
t204 = q_x.*q_z.*t96.*t102.*4.0e-9;
t205 = q_y.*q_z.*t96.*t102.*4.0e-9;
t206 = t6.*t96.*t102.*2.0e-9;
t207 = t7.*t96.*t102.*2.0e-9;
t208 = t8.*t96.*t102.*2.0e-9;
t209 = t9.*t96.*t102.*2.0e-9;
t210 = q_w.*q_x.*t102.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t213 = q_w.*q_x.*t96.*t104.*4.0e-9;
t214 = q_w.*q_y.*t102.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t216 = q_w.*q_y.*t96.*t104.*4.0e-9;
t217 = q_x.*q_z.*t102.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t219 = q_x.*q_z.*t96.*t104.*4.0e-9;
t220 = q_y.*q_z.*t102.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t222 = q_y.*q_z.*t96.*t104.*4.0e-9;
t223 = t6.*t102.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t225 = t6.*t96.*t104.*2.0e-9;
t226 = t7.*t102.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t229 = t7.*t96.*t104.*2.0e-9;
t230 = t8.*t102.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t233 = t8.*t96.*t104.*2.0e-9;
t234 = t9.*t102.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t236 = t9.*t96.*t104.*2.0e-9;
t240 = q_w.*q_x.*t104.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t242 = q_w.*q_y.*t102.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t245 = q_w.*q_y.*t104.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t246 = q_x.*q_z.*t102.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t249 = q_x.*q_z.*t104.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t250 = q_y.*q_z.*t102.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t253 = q_y.*q_z.*t104.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t254 = t6.*t102.*(t6+t7-t8-t18+t19+t29).*2.0e-9;
t257 = t6.*t104.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t261 = t7.*t104.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t266 = t8.*t104.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t268 = t9.*t102.*(t6+t7-t8-t18+t19+t29).*2.0e-9;
t271 = t9.*t104.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t276 = q_w.*q_y.*t104.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t279 = q_x.*q_z.*t104.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t282 = q_y.*q_z.*t104.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t285 = t6.*t104.*(t6+t7-t8-t18+t19+t29).*2.0e-9;
t292 = t9.*t104.*(t6+t7-t8-t18+t19+t29).*2.0e-9;
t294 = q_w.*q_x.*t102.*t104.*4.0e-9;
t296 = q_w.*q_y.*t102.*t104.*4.0e-9;
t298 = q_x.*q_z.*t102.*t104.*4.0e-9;
t300 = q_y.*q_z.*t102.*t104.*4.0e-9;
t305 = t6.*t102.*t104.*2.0e-9;
t307 = t7.*t102.*t104.*2.0e-9;
t309 = t8.*t102.*t104.*2.0e-9;
t311 = t9.*t102.*t104.*2.0e-9;
t364 = q_w.*q_x.*t119.*v_z.*8.0e-9;
t365 = q_w.*q_y.*t119.*v_z.*8.0e-9;
t366 = q_x.*q_z.*t119.*v_z.*8.0e-9;
t367 = q_y.*q_z.*t119.*v_z.*8.0e-9;
t374 = t6.*t119.*v_z.*4.0e-9;
t375 = t7.*t119.*v_z.*4.0e-9;
t376 = t8.*t119.*v_z.*4.0e-9;
t377 = t9.*t119.*v_z.*4.0e-9;
t382 = q_w.*q_x.*t30.*t119.*4.0e-9;
t383 = q_w.*q_x.*t31.*t119.*4.0e-9;
t384 = q_w.*q_y.*t30.*t119.*4.0e-9;
t385 = q_w.*q_y.*t31.*t119.*4.0e-9;
t386 = q_x.*q_z.*t30.*t119.*4.0e-9;
t387 = q_x.*q_z.*t31.*t119.*4.0e-9;
t388 = q_y.*q_z.*t30.*t119.*4.0e-9;
t389 = q_y.*q_z.*t31.*t119.*4.0e-9;
t390 = q_w.*q_x.*t32.*t119.*4.0e-9;
t391 = q_w.*q_y.*t32.*t119.*4.0e-9;
t392 = q_x.*q_z.*t32.*t119.*4.0e-9;
t393 = q_y.*q_z.*t32.*t119.*4.0e-9;
t394 = q_w.*q_x.*t33.*t119.*4.0e-9;
t396 = q_w.*q_x.*t34.*t119.*4.0e-9;
t398 = q_w.*q_y.*t33.*t119.*4.0e-9;
t400 = q_w.*q_y.*t34.*t119.*4.0e-9;
t401 = q_x.*q_z.*t33.*t119.*4.0e-9;
t403 = q_x.*q_z.*t34.*t119.*4.0e-9;
t404 = q_y.*q_z.*t33.*t119.*4.0e-9;
t406 = q_y.*q_z.*t34.*t119.*4.0e-9;
t407 = q_w.*q_x.*t35.*t119.*4.0e-9;
t409 = q_w.*q_y.*t35.*t119.*4.0e-9;
t410 = q_x.*q_z.*t35.*t119.*4.0e-9;
t411 = q_y.*q_z.*t35.*t119.*4.0e-9;
t412 = t6.*t30.*t119.*2.0e-9;
t413 = t6.*t31.*t119.*2.0e-9;
t414 = t7.*t30.*t119.*2.0e-9;
t415 = t7.*t31.*t119.*2.0e-9;
t416 = t8.*t30.*t119.*2.0e-9;
t417 = t8.*t31.*t119.*2.0e-9;
t418 = t9.*t30.*t119.*2.0e-9;
t419 = t9.*t31.*t119.*2.0e-9;
t420 = t6.*t32.*t119.*2.0e-9;
t421 = t7.*t32.*t119.*2.0e-9;
t422 = t8.*t32.*t119.*2.0e-9;
t423 = t9.*t32.*t119.*2.0e-9;
t435 = t6.*t33.*t119.*2.0e-9;
t437 = t6.*t34.*t119.*2.0e-9;
t438 = t7.*t33.*t119.*2.0e-9;
t440 = t7.*t34.*t119.*2.0e-9;
t442 = t8.*t33.*t119.*2.0e-9;
t444 = t8.*t34.*t119.*2.0e-9;
t446 = t9.*t33.*t119.*2.0e-9;
t448 = t9.*t34.*t119.*2.0e-9;
t449 = t6.*t35.*t119.*2.0e-9;
t450 = t7.*t35.*t119.*2.0e-9;
t452 = t8.*t35.*t119.*2.0e-9;
t454 = t9.*t35.*t119.*2.0e-9;
t465 = q_w.*t93.*t119.*4.0e-9;
t466 = q_x.*t93.*t119.*4.0e-9;
t467 = q_y.*t93.*t119.*4.0e-9;
t468 = q_z.*t93.*t119.*4.0e-9;
t472 = q_w.*t96.*t119.*4.0e-9;
t473 = q_x.*t96.*t119.*4.0e-9;
t474 = q_y.*t96.*t119.*4.0e-9;
t475 = q_z.*t96.*t119.*4.0e-9;
t476 = q_w.*t119.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t478 = q_x.*t119.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t480 = q_y.*t119.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t482 = q_z.*t119.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t483 = q_w.*t119.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t484 = q_x.*t119.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t485 = q_y.*t119.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t486 = q_z.*t119.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t487 = q_w.*t102.*t119.*4.0e-9;
t488 = q_x.*t102.*t119.*4.0e-9;
t489 = q_y.*t102.*t119.*4.0e-9;
t490 = q_z.*t102.*t119.*4.0e-9;
t493 = q_w.*t104.*t119.*4.0e-9;
t496 = q_x.*t104.*t119.*4.0e-9;
t499 = q_y.*t104.*t119.*4.0e-9;
t501 = q_z.*t104.*t119.*4.0e-9;
t514 = t124+t130;
t519 = t125+t128+t136+t138;
t108 = t103.^2;
t110 = t107.^2;
t153 = -t145;
t154 = q_w.*q_x.*t93.*t103.*4.0e-9;
t156 = q_w.*q_y.*t93.*t103.*4.0e-9;
t158 = q_x.*q_z.*t93.*t103.*4.0e-9;
t160 = q_y.*q_z.*t93.*t103.*4.0e-9;
t162 = t6.*t93.*t103.*2.0e-9;
t164 = -t150;
t165 = t7.*t93.*t103.*2.0e-9;
t167 = -t151;
t168 = t8.*t93.*t103.*2.0e-9;
t170 = t9.*t93.*t103.*2.0e-9;
t172 = q_w.*q_x.*t93.*t107.*4.0e-9;
t174 = -t155;
t175 = q_w.*q_y.*t93.*t107.*4.0e-9;
t176 = q_x.*q_z.*t93.*t107.*4.0e-9;
t177 = q_y.*q_z.*t93.*t107.*4.0e-9;
t178 = t6.*t93.*t107.*2.0e-9;
t179 = t7.*t93.*t107.*2.0e-9;
t181 = -t166;
t182 = t8.*t93.*t107.*2.0e-9;
t184 = -t169;
t185 = t9.*t93.*t107.*2.0e-9;
t211 = -t202;
t212 = q_w.*q_x.*t96.*t103.*4.0e-9;
t215 = q_w.*q_y.*t96.*t103.*4.0e-9;
t218 = q_x.*q_z.*t96.*t103.*4.0e-9;
t221 = q_y.*q_z.*t96.*t103.*4.0e-9;
t224 = t6.*t96.*t103.*2.0e-9;
t227 = -t207;
t228 = t7.*t96.*t103.*2.0e-9;
t231 = -t208;
t232 = t8.*t96.*t103.*2.0e-9;
t235 = t9.*t96.*t103.*2.0e-9;
t237 = q_w.*q_x.*t96.*t107.*4.0e-9;
t238 = q_w.*q_x.*t103.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t241 = -t213;
t243 = q_w.*q_y.*t96.*t107.*4.0e-9;
t244 = q_w.*q_y.*t103.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t247 = q_x.*q_z.*t96.*t107.*4.0e-9;
t248 = q_x.*q_z.*t103.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t251 = q_y.*q_z.*t96.*t107.*4.0e-9;
t252 = q_y.*q_z.*t103.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t255 = t6.*t96.*t107.*2.0e-9;
t256 = t6.*t103.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t258 = t7.*t96.*t107.*2.0e-9;
t259 = t7.*t103.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t262 = -t229;
t263 = t8.*t96.*t107.*2.0e-9;
t264 = t8.*t103.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t267 = -t233;
t269 = t9.*t96.*t107.*2.0e-9;
t270 = t9.*t103.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t272 = q_w.*q_x.*t107.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t274 = q_w.*q_y.*t107.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t275 = q_w.*q_y.*t103.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t277 = q_x.*q_z.*t107.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t278 = q_x.*q_z.*t103.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t280 = q_y.*q_z.*t107.*(t6+t7-t8-t18+t19+t29).*(-4.0e-9);
t281 = q_y.*q_z.*t103.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t283 = t6.*t107.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t284 = t6.*t103.*(t6+t7-t8-t18+t19+t29).*2.0e-9;
t286 = t7.*t107.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t288 = t8.*t107.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t290 = t9.*t107.*(t6+t7-t8-t18+t19+t29).*(-2.0e-9);
t291 = t9.*t103.*(t6+t7-t8-t18+t19+t29).*2.0e-9;
t293 = q_w.*q_x.*t102.*t103.*4.0e-9;
t295 = q_w.*q_y.*t102.*t103.*4.0e-9;
t297 = q_x.*q_z.*t102.*t103.*4.0e-9;
t299 = q_y.*q_z.*t102.*t103.*4.0e-9;
t301 = q_w.*q_y.*t107.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t302 = q_x.*q_z.*t107.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t303 = q_y.*q_z.*t107.*(t6+t7-t8-t18+t19+t29).*4.0e-9;
t304 = t6.*t102.*t103.*2.0e-9;
t306 = t7.*t102.*t103.*2.0e-9;
t308 = t8.*t102.*t103.*2.0e-9;
t310 = t9.*t102.*t103.*2.0e-9;
t312 = t6.*t107.*(t6+t7-t8-t18+t19+t29).*2.0e-9;
t313 = t9.*t107.*(t6+t7-t8-t18+t19+t29).*2.0e-9;
t314 = q_w.*q_x.*t102.*t107.*4.0e-9;
t316 = -t294;
t317 = q_w.*q_x.*t103.*t104.*4.0e-9;
t318 = q_w.*q_y.*t102.*t107.*4.0e-9;
t319 = q_w.*q_y.*t103.*t104.*4.0e-9;
t320 = q_x.*q_z.*t102.*t107.*4.0e-9;
t321 = q_x.*q_z.*t103.*t104.*4.0e-9;
t322 = q_y.*q_z.*t102.*t107.*4.0e-9;
t323 = q_y.*q_z.*t103.*t104.*4.0e-9;
t324 = t6.*t102.*t107.*2.0e-9;
t325 = t6.*t103.*t104.*2.0e-9;
t326 = t7.*t102.*t107.*2.0e-9;
t328 = -t307;
t329 = t7.*t103.*t104.*2.0e-9;
t330 = t8.*t102.*t107.*2.0e-9;
t332 = -t309;
t333 = t8.*t103.*t104.*2.0e-9;
t334 = t9.*t102.*t107.*2.0e-9;
t335 = t9.*t103.*t104.*2.0e-9;
t337 = q_w.*q_x.*t103.*t107.*4.0e-9;
t338 = q_w.*q_x.*t104.*t107.*4.0e-9;
t340 = q_w.*q_y.*t103.*t107.*4.0e-9;
t341 = q_w.*q_y.*t104.*t107.*4.0e-9;
t342 = q_x.*q_z.*t103.*t107.*4.0e-9;
t343 = q_x.*q_z.*t104.*t107.*4.0e-9;
t344 = q_y.*q_z.*t103.*t107.*4.0e-9;
t345 = q_y.*q_z.*t104.*t107.*4.0e-9;
t346 = t6.*t103.*t107.*2.0e-9;
t347 = t6.*t104.*t107.*2.0e-9;
t349 = t7.*t103.*t107.*2.0e-9;
t350 = t7.*t104.*t107.*2.0e-9;
t353 = t8.*t103.*t107.*2.0e-9;
t354 = t8.*t104.*t107.*2.0e-9;
t356 = t9.*t103.*t107.*2.0e-9;
t357 = t9.*t104.*t107.*2.0e-9;
t368 = t120.*2.0e-9;
t369 = -t364;
t370 = -t365;
t371 = -t366;
t372 = -t367;
t378 = -t374;
t379 = -t375;
t380 = -t376;
t381 = -t377;
t395 = -t382;
t397 = -t383;
t399 = -t384;
t402 = -t386;
t405 = -t388;
t408 = -t390;
t424 = -t394;
t425 = -t396;
t426 = -t398;
t427 = -t400;
t428 = -t401;
t429 = -t403;
t430 = -t404;
t431 = -t406;
t432 = -t409;
t433 = -t410;
t434 = -t411;
t436 = -t412;
t439 = -t414;
t441 = -t415;
t443 = -t416;
t445 = -t417;
t447 = -t418;
t451 = -t421;
t453 = -t422;
t455 = -t435;
t456 = -t437;
t457 = -t438;
t458 = -t440;
t459 = -t442;
t460 = -t444;
t461 = -t446;
t462 = -t448;
t463 = -t449;
t464 = -t454;
t469 = -t465;
t470 = -t466;
t471 = -t467;
t477 = -t472;
t479 = -t473;
t481 = -t474;
t491 = -t487;
t492 = q_w.*t103.*t119.*4.0e-9;
t494 = -t488;
t495 = q_x.*t103.*t119.*4.0e-9;
t497 = -t489;
t498 = q_y.*t103.*t119.*4.0e-9;
t500 = q_z.*t103.*t119.*4.0e-9;
t502 = q_w.*t107.*t119.*4.0e-9;
t503 = -t493;
t504 = q_x.*t107.*t119.*4.0e-9;
t506 = -t496;
t507 = q_y.*t107.*t119.*4.0e-9;
t509 = -t499;
t510 = q_z.*t107.*t119.*4.0e-9;
t522 = t87+t157+t159+t385+t387+t466;
t523 = t88+t146+t147+t400+t403+t467;
t530 = t82+t203+t204+t384+t386+t474;
t532 = t79+t216+t219+t398+t401+t473;
t540 = t86+t242+t246+t398+t401+t485;
t173 = -t154;
t180 = -t165;
t183 = -t168;
t186 = -t172;
t187 = -t179;
t188 = -t182;
t239 = -t212;
t260 = -t228;
t265 = -t232;
t273 = -t237;
t287 = -t258;
t289 = -t263;
t315 = -t293;
t327 = -t306;
t331 = -t308;
t336 = -t314;
t339 = -t317;
t348 = -t326;
t351 = -t329;
t352 = -t330;
t355 = -t333;
t358 = -t337;
t359 = -t338;
t360 = -t349;
t361 = -t350;
t362 = -t353;
t363 = -t354;
t373 = -t368;
t505 = -t495;
t508 = -t498;
t511 = -t502;
t512 = -t504;
t524 = t89+t156+t158+t384+t386+t468;
t526 = t80+t148+t153+t406+t425+t470;
t527 = t88+t161+t174+t389+t397+t467;
t528 = t86+t175+t176+t426+t428+t465;
t531 = t86+t205+t211+t388+t395+t479;
t533 = t87+t243+t247+t385+t387+t472;
t534 = t82+t222+t241+t404+t424+t474;
t537 = t88+t215+t218+t427+t429+t475;
t538 = t88+t238+t281+t389+t397+t476;
t539 = t89+t210+t250+t404+t424+t478;
t541 = t86+t240+t282+t382+t405+t485;
t542 = t87+t275+t278+t385+t387+t486;
t543 = t82+t276+t279+t399+t402+t484;
t544 = t81+t301+t302+t400+t403+t483;
t545 = t87+t272+t303+t406+t425+t486;
t546 = t295+t297+t365+t366+t490+t498;
t547 = t341+t343+t365+t366+t493+t504;
t550 = t84+t296+t298+t432+t433+t488+t499;
t551 = t90+t300+t316+t407+t434+t489+t506;
t552 = t91+t340+t342+t391+t392+t492+t510;
t554 = t86+t149+t152+t164+t167+t437+t448+t458+t460+t465;
t556 = t89+t163+t171+t181+t184+t413+t419+t441+t445+t468;
t558 = t92+t111+t112+t113+t114+t319+t321+t365+t366+t368+t495+t501;
t559 = t92+t111+t112+t113+t114+t318+t320+t368+t370+t371+t487+t507;
t562 = t87+t206+t209+t227+t231+t412+t418+t439+t443+t472;
t563 = t88+t225+t236+t262+t267+t435+t446+t457+t459+t475;
t565 = t81+t226+t230+t254+t268+t435+t446+t457+t459+t483;
t566 = t89+t259+t264+t284+t291+t413+t419+t441+t445+t478;
t568 = t87+t261+t266+t285+t292+t414+t416+t436+t447+t486;
t569 = t79+t286+t288+t312+t313+t437+t448+t458+t460+t480;
t574 = t305+t311+t328+t332+t450+t452+t463+t464+t490+t493;
t525 = t79+t160+t173+t388+t395+t469;
t529 = t89+t177+t186+t394+t430+t468;
t535 = t80+t221+t239+t396+t431+t477;
t536 = t88+t251+t273+t389+t397+t475;
t548 = t323+t339+t367+t369+t498+t503;
t549 = t322+t336+t364+t372+t490+t512;
t553 = t90+t344+t358+t393+t408+t500+t511;
t555 = t80+t162+t170+t180+t183+t412+t418+t439+t443+t470;
t557 = t81+t178+t185+t187+t188+t438+t442+t455+t461+t471;
t560 = t85+t115+t116+t117+t118+t299+t315+t367+t369+t373+t491+t505;
t561 = t92+t111+t112+t113+t114+t345+t359+t367+t368+t369+t501+t507;
t564 = t86+t224+t235+t260+t265+t440+t444+t456+t462+t479;
t567 = t89+t255+t269+t287+t289+t413+t419+t441+t445+t481;
t570 = t91+t304+t310+t327+t331+t374+t377+t379+t380+t492+t494;
t571 = t83+t324+t334+t348+t352+t375+t376+t378+t381+t497+t502;
t572 = t90+t325+t335+t351+t355+t374+t377+t379+t380+t500+t506;
t573 = t91+t347+t357+t361+t363+t374+t377+t379+t380+t509+t510;
t575 = t346+t356+t360+t362+t420+t423+t451+t453+t508+t512;
H_x = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,q_w.*q_y.*t100.*4.0e-9+q_x.*q_z.*t100.*4.0e-9,t518,t516,t540,t542,t544,t543,0.0,0.0,0.0,0.0,0.0,0.0,t518,q_w.*q_y.*t98.*4.0e-9+q_x.*q_z.*t98.*4.0e-9,t513,t530,t537,t533,t532,0.0,0.0,0.0,0.0,0.0,0.0,t516,t513,q_w.*q_y.*t95.*4.0e-9+q_x.*q_z.*t95.*4.0e-9,t523,t524,t528,t522,0.0,0.0,0.0,0.0,0.0,0.0,t540,t530,t523,t90+t391+t392+q_w.*q_y.*t105.*4.0e-9+q_x.*q_z.*t105.*4.0e-9+q_y.*t102.*t119.*8.0e-9,t546,t559,t550,0.0,0.0,0.0,0.0,0.0,0.0,t542,t537,t524,t546,t90+t409+t410+q_w.*q_y.*t108.*4.0e-9+q_x.*q_z.*t108.*4.0e-9+q_z.*t103.*t119.*8.0e-9,t552,t558,0.0,0.0,0.0,0.0,0.0,0.0,t544,t533,t528,t559,t552,t83+t432+t433+q_w.*q_y.*t110.*4.0e-9+q_x.*q_z.*t110.*4.0e-9+q_w.*t107.*t119.*8.0e-9,t547,0.0,0.0,0.0,0.0,0.0,0.0,t543,t532,t522,t550,t558,t547,t83-t391-t392+q_w.*q_y.*t109.*4.0e-9+q_x.*q_z.*t109.*4.0e-9+q_x.*t104.*t119.*8.0e-9,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,q_w.*q_x.*t100.*(-4.0e-9)+q_y.*q_z.*t100.*4.0e-9,t517,t515,t539,t538,t545,t541,0.0,0.0,0.0,0.0,0.0,0.0,t517,q_w.*q_x.*t98.*(-4.0e-9)+q_y.*q_z.*t98.*4.0e-9,t514,t531,t535,t536,t534,0.0,0.0,0.0,0.0,0.0,0.0,t515,t514,q_w.*q_x.*t95.*(-4.0e-9)+q_y.*q_z.*t95.*4.0e-9,t526,t525,t529,t527,0.0,0.0,0.0,0.0,0.0,0.0,t539,t531,t526,t91+t393+t408-q_w.*q_x.*t105.*4.0e-9+q_y.*q_z.*t105.*4.0e-9-q_x.*t102.*t119.*8.0e-9,t560,t549,t551,0.0,0.0,0.0,0.0,0.0,0.0,t538,t535,t525,t560,t84-t407+t411-q_w.*q_x.*t108.*4.0e-9+q_y.*q_z.*t108.*4.0e-9-q_w.*t103.*t119.*8.0e-9,t553,t548,0.0,0.0,0.0,0.0,0.0,0.0,t545,t536,t529,t549,t553,t91+t407+t434-q_w.*q_x.*t110.*4.0e-9+q_y.*q_z.*t110.*4.0e-9+q_z.*t107.*t119.*8.0e-9,t561,0.0,0.0,0.0,0.0,0.0,0.0,t541,t534,t527,t551,t548,t561,t84+t390-t393-q_w.*q_x.*t109.*4.0e-9+q_y.*q_z.*t109.*4.0e-9+q_y.*t104.*t119.*8.0e-9,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6.*t100.*2.0e-9-t7.*t100.*2.0e-9-t8.*t100.*2.0e-9+t9.*t100.*2.0e-9,t521,t520,t565,t566,t569,t568,0.0,0.0,0.0,0.0,0.0,0.0,t521,t6.*t98.*2.0e-9-t7.*t98.*2.0e-9-t8.*t98.*2.0e-9+t9.*t98.*2.0e-9,t519,t562,t564,t567,t563,0.0,0.0,0.0,0.0,0.0,0.0,t520,t519,t6.*t95.*2.0e-9-t7.*t95.*2.0e-9-t8.*t95.*2.0e-9+t9.*t95.*2.0e-9,t554,t555,t557,t556,0.0,0.0,0.0,0.0,0.0,0.0,t565,t562,t554,t92+t111+t112+t113+t114+t368+t420+t423+t451+t453+t6.*t105.*2.0e-9-t7.*t105.*2.0e-9-t8.*t105.*2.0e-9+t9.*t105.*2.0e-9+q_w.*t102.*t119.*8.0e-9,t570,t571,t574,0.0,0.0,0.0,0.0,0.0,0.0,t566,t564,t555,t570,t85+t115+t116+t117+t118+t373+t449-t450-t452+t454+t6.*t108.*2.0e-9-t7.*t108.*2.0e-9-t8.*t108.*2.0e-9+t9.*t108.*2.0e-9-q_x.*t103.*t119.*8.0e-9,t575,t572,0.0,0.0,0.0,0.0,0.0,0.0,t569,t567,t557,t571,t575,t85+t115+t116+t117+t118+t373+t450+t452+t463+t464+t6.*t110.*2.0e-9-t7.*t110.*2.0e-9-t8.*t110.*2.0e-9+t9.*t110.*2.0e-9-q_y.*t107.*t119.*8.0e-9,t573,0.0,0.0,0.0,0.0,0.0,0.0,t568,t563,t556,t574,t572,t573,t92+t111+t112+t113+t114+t368-t420+t421+t422-t423+t6.*t109.*2.0e-9-t7.*t109.*2.0e-9-t8.*t109.*2.0e-9+t9.*t109.*2.0e-9+q_z.*t104.*t119.*8.0e-9,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t63,t64,t50,t51,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t66,t65,t64,t63,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t50,t66,t48,t64,0.0,0.0,0.0,0.0,0.0,0.0,t63,t66,t50,t71,0.0,t58,t72,0.0,0.0,0.0,0.0,0.0,0.0,t64,t65,t66,0.0,t71,t72,t73,0.0,0.0,0.0,0.0,0.0,0.0,t50,t64,t48,t58,t72,t56,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t51,t63,t64,t72,t73,0.0,t56,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-9.7e+1./1.54e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-9.7e+1./1.54e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t51,t65,t64,t48,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t63,t49,t65,t51,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t64,t63,t66,t65,0.0,0.0,0.0,0.0,0.0,0.0,t51,t63,t64,t72,t73,0.0,t56,0.0,0.0,0.0,0.0,0.0,0.0,t65,t49,t63,t73,t57,t71,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t64,t65,t66,0.0,t71,t72,t73,0.0,0.0,0.0,0.0,0.0,0.0,t48,t51,t65,t56,0.0,t73,t57,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,9.7e+1./1.54e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,9.7e+1./1.54e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t69,t70,t67,t68,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t53,t52,t70,t69,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t67,t53,t54,t70,0.0,0.0,0.0,0.0,0.0,0.0,t69,t53,t67,t76,t60,t74,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t70,t52,t53,t60,t61,0.0,t74,0.0,0.0,0.0,0.0,0.0,0.0,t67,t70,t54,t74,0.0,t61,t75,0.0,0.0,0.0,0.0,0.0,0.0,t68,t69,t70,0.0,t74,t75,t76,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[13,13,13]);