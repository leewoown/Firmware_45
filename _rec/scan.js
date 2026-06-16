const fs = require('fs');
const path = 'd:/45 168S1P_Unit_PackBMSR03/F28069PackBMS/SysInclude/parameter.h';
const c = fs.readFileSync(path, 'utf8');
const lines = c.split(/\r?\n/);
// 占 = U+5360 (double-mojibake, irreversible) ; 寃/諛/怨 등 = UTF8-as-CP949 type
const reJeom = /占/;                 // 占
const reUtf8asCp = /[孃諛怨컄쇱쏜]/; // 寃諛怨 etc (sample CJK/odd hangul)
let jeom = [], utf8cp = [], todos = 0;
for (let i = 0; i < lines.length; i++) {
  const isTodos = /TODOS/.test(lines[i]);
  if (reJeom.test(lines[i])) jeom.push((i+1) + (isTodos?'(TODOS)':''));
  if (reUtf8asCp.test(lines[i])) utf8cp.push((i+1) + (isTodos?'(TODOS)':''));
}
console.log('占형(비가역) 줄:', jeom.length, '->', jeom.join(','));
console.log('寃利형(UTF8-as-CP949) 줄:', utf8cp.length, '->', utf8cp.join(','));
