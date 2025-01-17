function H_x = H_x_calc(q_w,q_x,q_y,q_z,u1,u2,u3,u4,v_x,v_y,v_z)
%H_X_CALC
%    H_X = H_X_CALC(Q_W,Q_X,Q_Y,Q_Z,U1,U2,U3,U4,V_X,V_Y,V_Z)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    03-Dec-2020 17:08:19

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
t30 = q_w.*(4.0./1.3e+1);
t31 = q_x.*(4.0./1.3e+1);
t32 = q_y.*(4.0./1.3e+1);
t33 = q_z.*(4.0./1.3e+1);
t34 = v_x.*(4.0./1.3e+1);
t35 = v_y.*(4.0./1.3e+1);
t40 = q_w.*(1.2e+1./1.3e+1);
t41 = q_w./7.7e+1;
t42 = q_w.*(2.0./2.51e+2);
t44 = q_x.*(1.2e+1./1.3e+1);
t45 = q_x./7.7e+1;
t46 = q_x.*(2.0./2.51e+2);
t48 = q_y.*(1.2e+1./1.3e+1);
t49 = q_y./7.7e+1;
t50 = q_y.*(2.0./2.51e+2);
t52 = q_z.*(1.2e+1./1.3e+1);
t53 = q_z./7.7e+1;
t54 = q_z.*(2.0./2.51e+2);
t56 = v_x./7.7e+1;
t57 = v_x.*(2.0./2.51e+2);
t59 = v_y./7.7e+1;
t60 = v_y.*(2.0./2.51e+2);
t61 = v_z.*(1.2e+1./1.3e+1);
t62 = v_z./7.7e+1;
t63 = v_z.*(2.0./2.51e+2);
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
t36 = t2+t5;
t37 = t3+t4;
t38 = t14+t15;
t39 = -t30;
t43 = -t31;
t47 = -t32;
t51 = -t33;
t55 = -t34;
t58 = -t35;
t67 = -t40;
t68 = -t41;
t69 = -t42;
t70 = -t44;
t71 = -t45;
t72 = -t46;
t73 = -t48;
t74 = -t49;
t75 = -t50;
t76 = -t52;
t77 = -t53;
t78 = -t54;
t79 = -t56;
t80 = -t57;
t81 = -t59;
t82 = -t60;
t83 = -t61;
t84 = -t62;
t85 = -t63;
t119 = t10.*2.7224e-8;
t120 = t11.*2.7224e-8;
t121 = t12.*2.7224e-8;
t122 = t13.*2.7224e-8;
t64 = t2+t23;
t65 = t3+t22;
t66 = t14+t24;
t86 = t36.*v_x;
t87 = t37.*v_x;
t88 = t36.*v_y;
t89 = t37.*v_y;
t90 = t36.*v_z;
t91 = t37.*v_z;
t101 = t16+t20+t21+t25;
t104 = t6+t8+t18+t19+t28+t29;
t105 = t8+t9+t18+t26+t27+t28;
t108 = (t6+t7-t8-t18+t19+t29).^2;
t109 = -v_x.*(t6+t7-t8-t18+t19+t29);
t114 = v_x.*(t6+t7-t8-t18+t19+t29);
t123 = -t119;
t124 = -t120;
t125 = -t121;
t126 = -t122;
t92 = t64.*v_x;
t93 = t65.*v_x;
t94 = t64.*v_y;
t95 = t65.*v_y;
t96 = t64.*v_z;
t97 = t65.*v_z;
t98 = -t86;
t102 = t101.*v_z;
t103 = t101.^2;
t106 = t104.^2;
t107 = t104.*v_y;
t127 = q_w.*q_x.*t101.*t104.*(9.0./2.5e+2);
t128 = q_w.*q_y.*t101.*t104.*(9.0./2.5e+2);
t129 = q_x.*q_z.*t101.*t104.*(9.0./2.5e+2);
t130 = q_y.*q_z.*t101.*t104.*(9.0./2.5e+2);
t131 = t6.*t101.*t104.*(9.0./5.0e+2);
t132 = t7.*t101.*t104.*(9.0./5.0e+2);
t133 = t8.*t101.*t104.*(9.0./5.0e+2);
t134 = t9.*t101.*t104.*(9.0./5.0e+2);
t135 = q_w.*q_x.*t101.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t137 = q_w.*q_y.*t101.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t138 = q_x.*q_z.*t101.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t139 = q_y.*q_z.*t101.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t140 = t6.*t101.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t141 = t7.*t101.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t143 = t8.*t101.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t145 = t9.*t101.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t146 = q_w.*q_y.*t101.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t147 = q_x.*q_z.*t101.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t148 = q_y.*q_z.*t101.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t149 = t6.*t101.*(t6+t7-t8-t18+t19+t29).*(9.0./5.0e+2);
t150 = t9.*t101.*(t6+t7-t8-t18+t19+t29).*(9.0./5.0e+2);
t195 = q_w.*q_x.*t104.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t196 = q_w.*q_y.*t104.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t197 = q_x.*q_z.*t104.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t198 = q_y.*q_z.*t104.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t199 = q_w.*q_y.*t104.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t200 = q_x.*q_z.*t104.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t201 = q_y.*q_z.*t104.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t202 = t6.*t104.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t203 = t7.*t104.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t204 = t8.*t104.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t205 = t9.*t104.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t210 = t6.*t104.*(t6+t7-t8-t18+t19+t29).*(9.0./5.0e+2);
t211 = t9.*t104.*(t6+t7-t8-t18+t19+t29).*(9.0./5.0e+2);
t99 = -t95;
t100 = -t96;
t110 = t88+t92+t97;
t112 = t91+t94+t98;
t136 = -t127;
t142 = -t132;
t144 = -t133;
t370 = t102+t107+t114;
t480 = t128+t129;
t482 = t135+t148;
t483 = t146+t147;
t525 = t195+t201;
t526 = t199+t200;
t528 = t141+t143+t149+t150;
t529 = t203+t204+t210+t211;
t111 = t87+t90+t99;
t113 = t110.^2;
t115 = t89+t93+t100;
t117 = t112.^2;
t151 = q_w.*q_x.*t101.*t110.*(9.0./2.5e+2);
t152 = q_w.*q_y.*t101.*t110.*(9.0./2.5e+2);
t153 = q_x.*q_z.*t101.*t110.*(9.0./2.5e+2);
t154 = q_y.*q_z.*t101.*t110.*(9.0./2.5e+2);
t155 = t6.*t101.*t110.*(9.0./5.0e+2);
t156 = t7.*t101.*t110.*(9.0./5.0e+2);
t157 = t8.*t101.*t110.*(9.0./5.0e+2);
t158 = t9.*t101.*t110.*(9.0./5.0e+2);
t161 = q_w.*q_x.*t101.*t112.*(9.0./2.5e+2);
t163 = q_w.*q_y.*t101.*t112.*(9.0./2.5e+2);
t165 = q_x.*q_z.*t101.*t112.*(9.0./2.5e+2);
t167 = q_y.*q_z.*t101.*t112.*(9.0./2.5e+2);
t169 = t6.*t101.*t112.*(9.0./5.0e+2);
t172 = t7.*t101.*t112.*(9.0./5.0e+2);
t175 = t8.*t101.*t112.*(9.0./5.0e+2);
t177 = t9.*t101.*t112.*(9.0./5.0e+2);
t206 = q_w.*q_x.*t104.*t110.*(9.0./2.5e+2);
t207 = q_w.*q_y.*t104.*t110.*(9.0./2.5e+2);
t208 = q_x.*q_z.*t104.*t110.*(9.0./2.5e+2);
t209 = q_y.*q_z.*t104.*t110.*(9.0./2.5e+2);
t212 = q_w.*q_x.*t110.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t215 = q_w.*q_x.*t104.*t112.*(9.0./2.5e+2);
t216 = q_w.*q_y.*t110.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t218 = q_w.*q_y.*t104.*t112.*(9.0./2.5e+2);
t219 = q_x.*q_z.*t110.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t221 = q_x.*q_z.*t104.*t112.*(9.0./2.5e+2);
t222 = q_y.*q_z.*t110.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t224 = q_y.*q_z.*t104.*t112.*(9.0./2.5e+2);
t225 = t6.*t104.*t110.*(9.0./5.0e+2);
t226 = t7.*t104.*t110.*(9.0./5.0e+2);
t227 = t8.*t104.*t110.*(9.0./5.0e+2);
t228 = t9.*t104.*t110.*(9.0./5.0e+2);
t232 = q_w.*q_x.*t112.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t234 = q_w.*q_y.*t110.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t237 = q_w.*q_y.*t112.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t238 = q_x.*q_z.*t110.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t241 = q_x.*q_z.*t112.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t242 = q_y.*q_z.*t110.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t245 = q_y.*q_z.*t112.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t246 = t6.*t110.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t248 = t6.*t104.*t112.*(9.0./5.0e+2);
t249 = t7.*t110.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t252 = t7.*t104.*t112.*(9.0./5.0e+2);
t253 = t8.*t110.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t256 = t8.*t104.*t112.*(9.0./5.0e+2);
t257 = t9.*t110.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t259 = t9.*t104.*t112.*(9.0./5.0e+2);
t264 = q_w.*q_y.*t112.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t267 = q_x.*q_z.*t112.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t270 = q_y.*q_z.*t112.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t271 = t6.*t110.*(t6+t7-t8-t18+t19+t29).*(9.0./5.0e+2);
t274 = t6.*t112.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t278 = t7.*t112.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t283 = t8.*t112.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t285 = t9.*t110.*(t6+t7-t8-t18+t19+t29).*(9.0./5.0e+2);
t288 = t9.*t112.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t290 = q_w.*q_x.*t110.*t112.*(9.0./2.5e+2);
t292 = q_w.*q_y.*t110.*t112.*(9.0./2.5e+2);
t294 = q_x.*q_z.*t110.*t112.*(9.0./2.5e+2);
t296 = q_y.*q_z.*t110.*t112.*(9.0./2.5e+2);
t302 = t6.*t112.*(t6+t7-t8-t18+t19+t29).*(9.0./5.0e+2);
t309 = t9.*t112.*(t6+t7-t8-t18+t19+t29).*(9.0./5.0e+2);
t321 = t6.*t110.*t112.*(9.0./5.0e+2);
t323 = t7.*t110.*t112.*(9.0./5.0e+2);
t325 = t8.*t110.*t112.*(9.0./5.0e+2);
t327 = t9.*t110.*t112.*(9.0./5.0e+2);
t371 = t370.^2;
t373 = q_w.*q_x.*t370.*v_z.*(9.0./1.25e+2);
t374 = q_w.*q_y.*t370.*v_z.*(9.0./1.25e+2);
t375 = q_x.*q_z.*t370.*v_z.*(9.0./1.25e+2);
t376 = q_y.*q_z.*t370.*v_z.*(9.0./1.25e+2);
t378 = t6.*t370.*v_z.*(9.0./2.5e+2);
t379 = t7.*t370.*v_z.*(9.0./2.5e+2);
t380 = t8.*t370.*v_z.*(9.0./2.5e+2);
t381 = t9.*t370.*v_z.*(9.0./2.5e+2);
t390 = q_w.*q_x.*t36.*t370.*(9.0./2.5e+2);
t391 = q_w.*q_x.*t37.*t370.*(9.0./2.5e+2);
t392 = q_w.*q_y.*t36.*t370.*(9.0./2.5e+2);
t393 = q_w.*q_y.*t37.*t370.*(9.0./2.5e+2);
t394 = q_x.*q_z.*t36.*t370.*(9.0./2.5e+2);
t395 = q_x.*q_z.*t37.*t370.*(9.0./2.5e+2);
t396 = q_y.*q_z.*t36.*t370.*(9.0./2.5e+2);
t397 = q_y.*q_z.*t37.*t370.*(9.0./2.5e+2);
t398 = q_w.*q_x.*t38.*t370.*(9.0./2.5e+2);
t399 = q_w.*q_y.*t38.*t370.*(9.0./2.5e+2);
t400 = q_x.*q_z.*t38.*t370.*(9.0./2.5e+2);
t401 = q_y.*q_z.*t38.*t370.*(9.0./2.5e+2);
t402 = t6.*t36.*t370.*(9.0./5.0e+2);
t403 = t6.*t37.*t370.*(9.0./5.0e+2);
t404 = t7.*t36.*t370.*(9.0./5.0e+2);
t405 = t7.*t37.*t370.*(9.0./5.0e+2);
t406 = t8.*t36.*t370.*(9.0./5.0e+2);
t407 = t8.*t37.*t370.*(9.0./5.0e+2);
t408 = t9.*t36.*t370.*(9.0./5.0e+2);
t409 = t9.*t37.*t370.*(9.0./5.0e+2);
t410 = t6.*t38.*t370.*(9.0./5.0e+2);
t411 = t7.*t38.*t370.*(9.0./5.0e+2);
t412 = t8.*t38.*t370.*(9.0./5.0e+2);
t413 = t9.*t38.*t370.*(9.0./5.0e+2);
t414 = q_w.*q_x.*t64.*t370.*(9.0./2.5e+2);
t416 = q_w.*q_x.*t65.*t370.*(9.0./2.5e+2);
t418 = q_w.*q_y.*t64.*t370.*(9.0./2.5e+2);
t420 = q_w.*q_y.*t65.*t370.*(9.0./2.5e+2);
t421 = q_x.*q_z.*t64.*t370.*(9.0./2.5e+2);
t423 = q_x.*q_z.*t65.*t370.*(9.0./2.5e+2);
t424 = q_y.*q_z.*t64.*t370.*(9.0./2.5e+2);
t426 = q_y.*q_z.*t65.*t370.*(9.0./2.5e+2);
t427 = q_w.*q_x.*t66.*t370.*(9.0./2.5e+2);
t429 = q_w.*q_y.*t66.*t370.*(9.0./2.5e+2);
t430 = q_x.*q_z.*t66.*t370.*(9.0./2.5e+2);
t431 = q_y.*q_z.*t66.*t370.*(9.0./2.5e+2);
t432 = t6.*t64.*t370.*(9.0./5.0e+2);
t434 = t6.*t65.*t370.*(9.0./5.0e+2);
t435 = t7.*t64.*t370.*(9.0./5.0e+2);
t437 = t7.*t65.*t370.*(9.0./5.0e+2);
t439 = t8.*t64.*t370.*(9.0./5.0e+2);
t441 = t8.*t65.*t370.*(9.0./5.0e+2);
t443 = t9.*t64.*t370.*(9.0./5.0e+2);
t445 = t9.*t65.*t370.*(9.0./5.0e+2);
t446 = t6.*t66.*t370.*(9.0./5.0e+2);
t447 = t7.*t66.*t370.*(9.0./5.0e+2);
t449 = t8.*t66.*t370.*(9.0./5.0e+2);
t451 = t9.*t66.*t370.*(9.0./5.0e+2);
t473 = q_w.*t101.*t370.*(9.0./2.5e+2);
t474 = q_x.*t101.*t370.*(9.0./2.5e+2);
t475 = q_y.*t101.*t370.*(9.0./2.5e+2);
t476 = q_z.*t101.*t370.*(9.0./2.5e+2);
t481 = t130+t136;
t484 = q_w.*t104.*t370.*(9.0./2.5e+2);
t485 = q_x.*t104.*t370.*(9.0./2.5e+2);
t486 = q_y.*t104.*t370.*(9.0./2.5e+2);
t487 = q_z.*t104.*t370.*(9.0./2.5e+2);
t488 = q_w.*t370.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t490 = q_x.*t370.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t492 = q_y.*t370.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t494 = q_z.*t370.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t495 = q_w.*t370.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t496 = q_x.*t370.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t497 = q_y.*t370.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t498 = q_z.*t370.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t499 = q_w.*t110.*t370.*(9.0./2.5e+2);
t500 = q_x.*t110.*t370.*(9.0./2.5e+2);
t501 = q_y.*t110.*t370.*(9.0./2.5e+2);
t502 = q_z.*t110.*t370.*(9.0./2.5e+2);
t505 = q_w.*t112.*t370.*(9.0./2.5e+2);
t508 = q_x.*t112.*t370.*(9.0./2.5e+2);
t511 = q_y.*t112.*t370.*(9.0./2.5e+2);
t513 = q_z.*t112.*t370.*(9.0./2.5e+2);
t527 = t131+t134+t142+t144;
t116 = t111.^2;
t118 = t115.^2;
t159 = -t151;
t160 = q_w.*q_x.*t101.*t111.*(9.0./2.5e+2);
t162 = q_w.*q_y.*t101.*t111.*(9.0./2.5e+2);
t164 = q_x.*q_z.*t101.*t111.*(9.0./2.5e+2);
t166 = q_y.*q_z.*t101.*t111.*(9.0./2.5e+2);
t168 = t6.*t101.*t111.*(9.0./5.0e+2);
t170 = -t156;
t171 = t7.*t101.*t111.*(9.0./5.0e+2);
t173 = -t157;
t174 = t8.*t101.*t111.*(9.0./5.0e+2);
t176 = t9.*t101.*t111.*(9.0./5.0e+2);
t178 = q_w.*q_x.*t101.*t115.*(9.0./2.5e+2);
t180 = -t161;
t181 = q_w.*q_y.*t101.*t115.*(9.0./2.5e+2);
t182 = q_x.*q_z.*t101.*t115.*(9.0./2.5e+2);
t183 = q_y.*q_z.*t101.*t115.*(9.0./2.5e+2);
t184 = t6.*t101.*t115.*(9.0./5.0e+2);
t185 = t7.*t101.*t115.*(9.0./5.0e+2);
t187 = -t172;
t188 = t8.*t101.*t115.*(9.0./5.0e+2);
t190 = -t175;
t191 = t9.*t101.*t115.*(9.0./5.0e+2);
t213 = -t206;
t214 = q_w.*q_x.*t104.*t111.*(9.0./2.5e+2);
t217 = q_w.*q_y.*t104.*t111.*(9.0./2.5e+2);
t220 = q_x.*q_z.*t104.*t111.*(9.0./2.5e+2);
t223 = q_y.*q_z.*t104.*t111.*(9.0./2.5e+2);
t229 = q_w.*q_x.*t104.*t115.*(9.0./2.5e+2);
t230 = q_w.*q_x.*t111.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t233 = -t215;
t235 = q_w.*q_y.*t104.*t115.*(9.0./2.5e+2);
t236 = q_w.*q_y.*t111.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t239 = q_x.*q_z.*t104.*t115.*(9.0./2.5e+2);
t240 = q_x.*q_z.*t111.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t243 = q_y.*q_z.*t104.*t115.*(9.0./2.5e+2);
t244 = q_y.*q_z.*t111.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t247 = t6.*t104.*t111.*(9.0./5.0e+2);
t250 = -t226;
t251 = t7.*t104.*t111.*(9.0./5.0e+2);
t254 = -t227;
t255 = t8.*t104.*t111.*(9.0./5.0e+2);
t258 = t9.*t104.*t111.*(9.0./5.0e+2);
t260 = q_w.*q_x.*t115.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t262 = q_w.*q_y.*t115.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t263 = q_w.*q_y.*t111.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t265 = q_x.*q_z.*t115.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t266 = q_x.*q_z.*t111.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t268 = q_y.*q_z.*t115.*(t6+t7-t8-t18+t19+t29).*(-9.0./2.5e+2);
t269 = q_y.*q_z.*t111.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t272 = t6.*t104.*t115.*(9.0./5.0e+2);
t273 = t6.*t111.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t275 = t7.*t104.*t115.*(9.0./5.0e+2);
t276 = t7.*t111.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t279 = -t252;
t280 = t8.*t104.*t115.*(9.0./5.0e+2);
t281 = t8.*t111.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t284 = -t256;
t286 = t9.*t104.*t115.*(9.0./5.0e+2);
t287 = t9.*t111.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t289 = q_w.*q_x.*t110.*t111.*(9.0./2.5e+2);
t291 = q_w.*q_y.*t110.*t111.*(9.0./2.5e+2);
t293 = q_x.*q_z.*t110.*t111.*(9.0./2.5e+2);
t295 = q_y.*q_z.*t110.*t111.*(9.0./2.5e+2);
t297 = q_w.*q_y.*t115.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t298 = q_x.*q_z.*t115.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t299 = q_y.*q_z.*t115.*(t6+t7-t8-t18+t19+t29).*(9.0./2.5e+2);
t300 = t6.*t115.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t301 = t6.*t111.*(t6+t7-t8-t18+t19+t29).*(9.0./5.0e+2);
t303 = t7.*t115.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t305 = t8.*t115.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t307 = t9.*t115.*(t6+t7-t8-t18+t19+t29).*(-9.0./5.0e+2);
t308 = t9.*t111.*(t6+t7-t8-t18+t19+t29).*(9.0./5.0e+2);
t310 = q_w.*q_x.*t110.*t115.*(9.0./2.5e+2);
t312 = -t290;
t313 = q_w.*q_x.*t111.*t112.*(9.0./2.5e+2);
t314 = q_w.*q_y.*t110.*t115.*(9.0./2.5e+2);
t315 = q_w.*q_y.*t111.*t112.*(9.0./2.5e+2);
t316 = q_x.*q_z.*t110.*t115.*(9.0./2.5e+2);
t317 = q_x.*q_z.*t111.*t112.*(9.0./2.5e+2);
t318 = q_y.*q_z.*t110.*t115.*(9.0./2.5e+2);
t319 = q_y.*q_z.*t111.*t112.*(9.0./2.5e+2);
t320 = t6.*t110.*t111.*(9.0./5.0e+2);
t322 = t7.*t110.*t111.*(9.0./5.0e+2);
t324 = t8.*t110.*t111.*(9.0./5.0e+2);
t326 = t9.*t110.*t111.*(9.0./5.0e+2);
t328 = t6.*t115.*(t6+t7-t8-t18+t19+t29).*(9.0./5.0e+2);
t329 = t9.*t115.*(t6+t7-t8-t18+t19+t29).*(9.0./5.0e+2);
t331 = q_w.*q_x.*t111.*t115.*(9.0./2.5e+2);
t332 = q_w.*q_x.*t112.*t115.*(9.0./2.5e+2);
t334 = q_w.*q_y.*t111.*t115.*(9.0./2.5e+2);
t335 = q_w.*q_y.*t112.*t115.*(9.0./2.5e+2);
t336 = q_x.*q_z.*t111.*t115.*(9.0./2.5e+2);
t337 = q_x.*q_z.*t112.*t115.*(9.0./2.5e+2);
t338 = q_y.*q_z.*t111.*t115.*(9.0./2.5e+2);
t339 = q_y.*q_z.*t112.*t115.*(9.0./2.5e+2);
t340 = t6.*t110.*t115.*(9.0./5.0e+2);
t341 = t6.*t111.*t112.*(9.0./5.0e+2);
t342 = t7.*t110.*t115.*(9.0./5.0e+2);
t344 = -t323;
t345 = t7.*t111.*t112.*(9.0./5.0e+2);
t346 = t8.*t110.*t115.*(9.0./5.0e+2);
t348 = -t325;
t349 = t8.*t111.*t112.*(9.0./5.0e+2);
t350 = t9.*t110.*t115.*(9.0./5.0e+2);
t351 = t9.*t111.*t112.*(9.0./5.0e+2);
t354 = t6.*t111.*t115.*(9.0./5.0e+2);
t355 = t6.*t112.*t115.*(9.0./5.0e+2);
t357 = t7.*t111.*t115.*(9.0./5.0e+2);
t358 = t7.*t112.*t115.*(9.0./5.0e+2);
t361 = t8.*t111.*t115.*(9.0./5.0e+2);
t362 = t8.*t112.*t115.*(9.0./5.0e+2);
t364 = t9.*t111.*t115.*(9.0./5.0e+2);
t365 = t9.*t112.*t115.*(9.0./5.0e+2);
t372 = t371.*(9.0./5.0e+2);
t382 = -t373;
t383 = -t374;
t384 = -t375;
t385 = -t376;
t386 = -t378;
t387 = -t379;
t388 = -t380;
t389 = -t381;
t415 = -t390;
t417 = -t391;
t419 = -t392;
t422 = -t394;
t425 = -t396;
t428 = -t398;
t433 = -t402;
t436 = -t404;
t438 = -t405;
t440 = -t406;
t442 = -t407;
t444 = -t408;
t448 = -t411;
t450 = -t412;
t452 = -t414;
t453 = -t416;
t454 = -t418;
t455 = -t420;
t456 = -t421;
t457 = -t423;
t458 = -t424;
t459 = -t426;
t460 = -t429;
t461 = -t430;
t462 = -t431;
t463 = -t432;
t464 = -t434;
t465 = -t435;
t466 = -t437;
t467 = -t439;
t468 = -t441;
t469 = -t443;
t470 = -t445;
t471 = -t446;
t472 = -t451;
t477 = -t473;
t478 = -t474;
t479 = -t475;
t489 = -t484;
t491 = -t485;
t493 = -t486;
t503 = -t499;
t504 = q_w.*t111.*t370.*(9.0./2.5e+2);
t506 = -t500;
t507 = q_x.*t111.*t370.*(9.0./2.5e+2);
t509 = -t501;
t510 = q_y.*t111.*t370.*(9.0./2.5e+2);
t512 = q_z.*t111.*t370.*(9.0./2.5e+2);
t514 = q_w.*t115.*t370.*(9.0./2.5e+2);
t515 = -t505;
t516 = q_x.*t115.*t370.*(9.0./2.5e+2);
t518 = -t508;
t519 = q_y.*t115.*t370.*(9.0./2.5e+2);
t521 = -t511;
t522 = q_z.*t115.*t370.*(9.0./2.5e+2);
t530 = t70+t163+t165+t393+t395+t474;
t531 = t73+t152+t153+t420+t423+t475;
t538 = t33+t207+t208+t392+t394+t486;
t540 = t30+t218+t221+t418+t421+t485;
t548 = t39+t234+t238+t418+t421+t497;
t179 = -t160;
t186 = -t171;
t189 = -t174;
t192 = -t178;
t193 = -t185;
t194 = -t188;
t231 = -t214;
t261 = -t229;
t277 = -t251;
t282 = -t255;
t304 = -t275;
t306 = -t280;
t311 = -t289;
t330 = -t310;
t333 = -t313;
t343 = -t322;
t347 = -t324;
t352 = -t331;
t353 = -t332;
t356 = -t342;
t359 = -t345;
t360 = -t346;
t363 = -t349;
t366 = -t357;
t367 = -t358;
t368 = -t361;
t369 = -t362;
t377 = -t372;
t517 = -t507;
t520 = -t510;
t523 = -t514;
t524 = -t516;
t532 = t76+t162+t164+t392+t394+t476;
t534 = t44+t154+t159+t426+t453+t478;
t535 = t73+t167+t180+t397+t417+t475;
t536 = t67+t181+t182+t454+t456+t473;
t539 = t39+t209+t213+t396+t415+t491;
t541 = t43+t235+t239+t393+t395+t484;
t542 = t33+t224+t233+t424+t452+t486;
t545 = t47+t217+t220+t455+t457+t487;
t546 = t47+t230+t269+t397+t417+t488;
t547 = t51+t212+t242+t424+t452+t490;
t549 = t39+t232+t270+t390+t425+t497;
t550 = t43+t263+t266+t393+t395+t498;
t551 = t33+t264+t267+t419+t422+t496;
t552 = t32+t297+t298+t420+t423+t495;
t553 = t43+t260+t299+t426+t453+t498;
t554 = t291+t293+t374+t375+t502+t510;
t555 = t335+t337+t374+t375+t505+t516;
t558 = t35+t292+t294+t460+t461+t500+t511;
t559 = t55+t296+t312+t427+t462+t501+t518;
t560 = t58+t334+t336+t399+t400+t504+t522;
t562 = t67+t155+t158+t170+t173+t434+t445+t466+t468+t473;
t564 = t76+t169+t177+t187+t190+t403+t409+t438+t442+t476;
t566 = t43+t225+t228+t250+t254+t402+t408+t436+t440+t484;
t567 = t47+t248+t259+t279+t284+t432+t443+t465+t467+t487;
t569 = t32+t249+t253+t271+t285+t432+t443+t465+t467+t495;
t570 = t51+t276+t281+t301+t308+t403+t409+t438+t442+t490;
t572 = t43+t278+t283+t302+t309+t404+t406+t433+t444+t498;
t573 = t30+t303+t305+t328+t329+t434+t445+t466+t468+t492;
t574 = t83+t119+t120+t121+t122+t315+t317+t372+t374+t375+t507+t513;
t575 = t83+t119+t120+t121+t122+t314+t316+t372+t383+t384+t499+t519;
t582 = t321+t327+t344+t348+t447+t449+t471+t472+t502+t505;
t533 = t40+t166+t179+t396+t415+t477;
t537 = t76+t183+t192+t414+t458+t476;
t543 = t31+t223+t231+t416+t459+t489;
t544 = t47+t243+t261+t397+t417+t487;
t556 = t319+t333+t376+t382+t510+t515;
t557 = t318+t330+t373+t385+t502+t524;
t561 = t55+t338+t352+t401+t428+t512+t523;
t563 = t44+t168+t176+t186+t189+t402+t408+t436+t440+t478;
t565 = t48+t184+t191+t193+t194+t435+t439+t463+t469+t479;
t568 = t39+t247+t258+t277+t282+t437+t441+t464+t470+t491;
t571 = t51+t272+t286+t304+t306+t403+t409+t438+t442+t493;
t576 = t61+t123+t124+t125+t126+t295+t311+t376+t377+t382+t503+t517;
t577 = t83+t119+t120+t121+t122+t339+t353+t372+t376+t382+t513+t519;
t578 = t58+t320+t326+t343+t347+t378+t381+t387+t388+t504+t506;
t579 = t34+t340+t350+t356+t360+t379+t380+t386+t389+t509+t514;
t580 = t55+t341+t351+t359+t363+t378+t381+t387+t388+t512+t518;
t581 = t58+t355+t365+t367+t369+t378+t381+t387+t388+t521+t522;
t583 = t354+t364+t366+t368+t410+t413+t448+t450+t520+t524;
H_x = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,q_w.*q_y.*t108.*(9.0./2.5e+2)+q_x.*q_z.*t108.*(9.0./2.5e+2),t526,t483,t548,t550,t552,t551,0.0,0.0,0.0,0.0,0.0,0.0,t526,q_w.*q_y.*t106.*(9.0./2.5e+2)+q_x.*q_z.*t106.*(9.0./2.5e+2),t480,t538,t545,t541,t540,0.0,0.0,0.0,0.0,0.0,0.0,t483,t480,q_w.*q_y.*t103.*(9.0./2.5e+2)+q_x.*q_z.*t103.*(9.0./2.5e+2),t531,t532,t536,t530,0.0,0.0,0.0,0.0,0.0,0.0,t548,t538,t531,t55+t399+t400+q_w.*q_y.*t113.*(9.0./2.5e+2)+q_x.*q_z.*t113.*(9.0./2.5e+2)+q_y.*t110.*t370.*(9.0./1.25e+2),t554,t575,t558,0.0,0.0,0.0,0.0,0.0,0.0,t550,t545,t532,t554,t55+t429+t430+q_w.*q_y.*t116.*(9.0./2.5e+2)+q_x.*q_z.*t116.*(9.0./2.5e+2)+q_z.*t111.*t370.*(9.0./1.25e+2),t560,t574,0.0,0.0,0.0,0.0,0.0,0.0,t552,t541,t536,t575,t560,t34+t460+t461+q_w.*q_y.*t118.*(9.0./2.5e+2)+q_x.*q_z.*t118.*(9.0./2.5e+2)+q_w.*t115.*t370.*(9.0./1.25e+2),t555,0.0,0.0,0.0,0.0,0.0,0.0,t551,t540,t530,t558,t574,t555,t34-t399-t400+q_w.*q_y.*t117.*(9.0./2.5e+2)+q_x.*q_z.*t117.*(9.0./2.5e+2)+q_x.*t112.*t370.*(9.0./1.25e+2),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,q_w.*q_x.*t108.*(-9.0./2.5e+2)+q_y.*q_z.*t108.*(9.0./2.5e+2),t525,t482,t547,t546,t553,t549,0.0,0.0,0.0,0.0,0.0,0.0,t525,q_w.*q_x.*t106.*(-9.0./2.5e+2)+q_y.*q_z.*t106.*(9.0./2.5e+2),t481,t539,t543,t544,t542,0.0,0.0,0.0,0.0,0.0,0.0,t482,t481,q_w.*q_x.*t103.*(-9.0./2.5e+2)+q_y.*q_z.*t103.*(9.0./2.5e+2),t534,t533,t537,t535,0.0,0.0,0.0,0.0,0.0,0.0,t547,t539,t534,t58+t401+t428-q_w.*q_x.*t113.*(9.0./2.5e+2)+q_y.*q_z.*t113.*(9.0./2.5e+2)-q_x.*t110.*t370.*(9.0./1.25e+2),t576,t557,t559,0.0,0.0,0.0,0.0,0.0,0.0,t546,t543,t533,t576,t35-t427+t431-q_w.*q_x.*t116.*(9.0./2.5e+2)+q_y.*q_z.*t116.*(9.0./2.5e+2)-q_w.*t111.*t370.*(9.0./1.25e+2),t561,t556,0.0,0.0,0.0,0.0,0.0,0.0,t553,t544,t537,t557,t561,t58+t427+t462-q_w.*q_x.*t118.*(9.0./2.5e+2)+q_y.*q_z.*t118.*(9.0./2.5e+2)+q_z.*t115.*t370.*(9.0./1.25e+2),t577,0.0,0.0,0.0,0.0,0.0,0.0,t549,t542,t535,t559,t556,t577,t35+t398-t401-q_w.*q_x.*t117.*(9.0./2.5e+2)+q_y.*q_z.*t117.*(9.0./2.5e+2)+q_y.*t112.*t370.*(9.0./1.25e+2),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6.*t108.*(9.0./5.0e+2)-t7.*t108.*(9.0./5.0e+2)-t8.*t108.*(9.0./5.0e+2)+t9.*t108.*(9.0./5.0e+2),t529,t528,t569,t570,t573,t572,0.0,0.0,0.0,0.0,0.0,0.0,t529,t6.*t106.*(9.0./5.0e+2)-t7.*t106.*(9.0./5.0e+2)-t8.*t106.*(9.0./5.0e+2)+t9.*t106.*(9.0./5.0e+2),t527,t566,t568,t571,t567,0.0,0.0,0.0,0.0,0.0,0.0,t528,t527,t6.*t103.*(9.0./5.0e+2)-t7.*t103.*(9.0./5.0e+2)-t8.*t103.*(9.0./5.0e+2)+t9.*t103.*(9.0./5.0e+2),t562,t563,t565,t564,0.0,0.0,0.0,0.0,0.0,0.0,t569,t566,t562,t83+t119+t120+t121+t122+t372+t410+t413+t448+t450+t6.*t113.*(9.0./5.0e+2)-t7.*t113.*(9.0./5.0e+2)-t8.*t113.*(9.0./5.0e+2)+t9.*t113.*(9.0./5.0e+2)+q_w.*t110.*t370.*(9.0./1.25e+2),t578,t579,t582,0.0,0.0,0.0,0.0,0.0,0.0,t570,t568,t563,t578,t61+t123+t124+t125+t126+t377+t446-t447-t449+t451+t6.*t116.*(9.0./5.0e+2)-t7.*t116.*(9.0./5.0e+2)-t8.*t116.*(9.0./5.0e+2)+t9.*t116.*(9.0./5.0e+2)-q_x.*t111.*t370.*(9.0./1.25e+2),t583,t580,0.0,0.0,0.0,0.0,0.0,0.0,t573,t571,t565,t579,t583,t61+t123+t124+t125+t126+t377+t447+t449+t471+t472+t6.*t118.*(9.0./5.0e+2)-t7.*t118.*(9.0./5.0e+2)-t8.*t118.*(9.0./5.0e+2)+t9.*t118.*(9.0./5.0e+2)-q_y.*t115.*t370.*(9.0./1.25e+2),t581,0.0,0.0,0.0,0.0,0.0,0.0,t572,t567,t564,t582,t580,t581,t83+t119+t120+t121+t122+t372-t410+t411+t412-t413+t6.*t117.*(9.0./5.0e+2)-t7.*t117.*(9.0./5.0e+2)-t8.*t117.*(9.0./5.0e+2)+t9.*t117.*(9.0./5.0e+2)+q_z.*t112.*t370.*(9.0./1.25e+2),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t68,t71,t49,t53,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t77,t74,t71,t68,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t49,t77,t41,t71,0.0,0.0,0.0,0.0,0.0,0.0,t68,t77,t49,t79,0.0,t62,t81,0.0,0.0,0.0,0.0,0.0,0.0,t71,t74,t77,0.0,t79,t81,t84,0.0,0.0,0.0,0.0,0.0,0.0,t49,t71,t41,t62,t81,t56,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t53,t68,t71,t81,t84,0.0,t56,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-9.7e+1./1.54e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-9.7e+1./1.54e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t53,t74,t71,t41,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t68,t45,t74,t53,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t71,t68,t77,t74,0.0,0.0,0.0,0.0,0.0,0.0,t53,t68,t71,t81,t84,0.0,t56,0.0,0.0,0.0,0.0,0.0,0.0,t74,t45,t68,t84,t59,t79,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t71,t74,t77,0.0,t79,t81,t84,0.0,0.0,0.0,0.0,0.0,0.0,t41,t53,t74,t56,0.0,t84,t59,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,9.7e+1./1.54e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,9.7e+1./1.54e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t75,t78,t69,t72,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t46,t42,t78,t75,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t69,t46,t50,t78,0.0,0.0,0.0,0.0,0.0,0.0,t75,t46,t69,t85,t60,t80,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t78,t42,t46,t60,t63,0.0,t80,0.0,0.0,0.0,0.0,0.0,0.0,t69,t78,t50,t80,0.0,t63,t82,0.0,0.0,0.0,0.0,0.0,0.0,t72,t75,t78,0.0,t80,t82,t85,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[13,13,13]);
