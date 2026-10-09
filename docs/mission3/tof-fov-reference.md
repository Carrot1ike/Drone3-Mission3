# ToF 화각: 공개 부품값과 현재 patch의 구분

2026-10-08 추가 제조사 원문 대조. 새 정보는 기존 실험 원자료·profile·서명을 바꾸지 않는다. 현재 entry2 검증의 baseline profile은 그대로 고정하고, 별도 제조사 부품 reference sensitivity를 둔다.

[ST AN5191 Rev1](https://www.st.com/resource/en/application_note/an5191-using-the-programmable-region-of-interest-roi-with-the-vl53l1x-stmicroelectronics.pdf)의인쇄p4는16×16 수신SPAD의전체**대각**화각27°를설명한다. p7 표는4×4=15°,8×8=20°,16×16=27°의대각화각예시를제시한다. 이는부품광학P이며 실제지급deck장착·calibration·반사율별유효범위와동일하지않다. emission27°cone과receiver의대각화각도구분해야한다.

공식 firmware 고정commit `f6e0f0a3...`에서zranger2는DataInit/StaticInit뒤MEDIUM·25msbudget·25mstask를사용한다. 조사한deck/wrapper에SetUserROI호출이없고STpreset의`roi_config__user_roi_requested_global_xy_size=0xFF`주석은16×16이다(C). 실제장치NVM·revision·장착·offset·cross-talk측정은U다.

현재 `patch_sensor(half_angle,side)`는 x/y에동일tan-grid를놓는 **정사각각도patch**다. 원형원뿔외곽에서ray를제외하지않는다. 따라서기존ToF `half_angle=7.5°`는각축전체15°이고,핀홀가정의대각은약21.095°다. 이를실제full27°FoV또는원형full15°cone이라고표시하면안된다.7.5°는펌웨어nearestreturn기울기모델상수C에서채택한기존S이며제조사조립광학측정값이아니다.

동일x/y정사각핀홀가정에서`half_axis=atan(tan(diagonal/2)/sqrt(2))`다. full diagonal27°는각축full약19.269°로도출된다(D). 이식은렌즈왜곡·SPAD중심·측정가중치가없는근사S이므로실측deck FoV를채운것은아니다.27°를각축H/V로쓰면대각이약37.51°가되어다른모델이된다.

새 `config/mission3-tof-27diag-sensitivity.json`은현재제어코드·entry·native camera·모터·접촉설정에서ToFpatch각축반각만이D값으로바꾼별도모델이다. 실제deck FoV null과nearest-return/ray-noise S를유지한다. 이전5개연구통과에이새profile을섞지않고,training/sensitivity로비교한뒤별도holdout이필요하다.

finite ray의최소거리반환은ST의반사율/광자수/target강도/offset/RangeStatus를재현하지않는다. 부품공칭화각을맞추는것과센서의실제mixedsurface오류모델을맞추는것은별개다. Flowpatch42°도해당PixArt렌즈typicalangle을기반으로한S이며실제조립deck의축별보정FoV는U다.
