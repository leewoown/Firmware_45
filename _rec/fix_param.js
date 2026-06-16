const fs = require('fs');
const path = 'd:/45 168S1P_Unit_PackBMSR03/F28069PackBMS/SysInclude/parameter.h';

// NN -> 정상 한글 내용 (대화 초반 grep에서 복원, 상태는 모두 검증)
const map = {
  32: 'Product_Type->SW_Type, 초기값 0 (BRA_SW_Type)',
  33: '방전 경고 과전류 70->150A (R01 반영)',
  34: '셀 경고 과온 55->45C (R01 반영)',
  35: '충전 셀 경고 저온 5->10C (R01 반영, 충/방전 값 교정)',
  36: '방전 셀 경고 저온 10->5C (R01 반영, 충/방전 값 교정)',
  37: 'R01 명시 Release Point 상수화',
  38: '방전 Fault 과전류 80->200A (R01 반영)',
  39: '셀 온도편차 Fault 15->13C (R01 반영)',
  40: 'R01 명시 Release Point 상수화',
  41: '방전 보호 과전류 100->250A (R01 반영)',
  42: '보호 SOC Low -0.1->0% (R01 반영)',
  43: '셀 전압편차 보호 0.5->0.4V (R01 반영)',
  44: '셀 보호 과온 55->54C (R01 반영)',
};

let content = fs.readFileSync(path, 'utf8');
const hadBom = content.charCodeAt(0) === 0xFEFF;
if (hadBom) content = content.slice(1);
const lines = content.split(/\r?\n/);
let fixed = 0;
const log = [];
for (let i = 0; i < lines.length; i++) {
  const m = lines[i].match(/\/\/ TODOS : \[.*?\] \((\d+),/);
  if (!m) continue;
  const nn = parseInt(m[1], 10);
  if (!(nn in map)) continue;
  const idx = lines[i].indexOf('// TODOS');
  const prefix = lines[i].substring(0, idx);
  lines[i] = prefix + `// TODOS : [검증] (${String(nn).padStart(2, '0')}, ${map[nn]})`;
  fixed++;
  log.push(`L${i + 1} NN=${nn} -> ${map[nn]}`);
}
fs.writeFileSync(path, lines.join('\r\n'), { encoding: 'utf8' });  // no BOM
console.log('fixed=' + fixed);
log.forEach(l => console.log('  ' + l));
