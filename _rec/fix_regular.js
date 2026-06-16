const fs = require('fs');
const root = 'd:/45 168S1P_Unit_PackBMSR03/F28069PackBMS';
const target = root + '/SysInclude/parameter.h';
const gitClean = root + '/_rec/param_git_utf8.txt';

const skel = s => s.replace(/[^\x00-\x7F]/g, '').replace(/\s+/g, ' ').trim();

// 1) 寃利형 3줄: 문맥 재구성 (전체 클린 라인, 1-based)
const reconstruct = {
  348: '// R01 Release(reset) Point - Alarm 히스테리시스',
  363: '// R01 경고/Fault 판정 Delay : 100ms @ 1ms(cpu_timer0) ISR -> 100 count',
  391: '// R01 Release(reset) Point - Fault 히스테리시스',
};

// 2) 占형 12줄: git 스켈레톤 매칭 교체
const jeomLines = [2, 6, 27, 30, 177, 215, 219, 223, 234, 235, 236, 320];

let cur = fs.readFileSync(target, 'utf8');
if (cur.charCodeAt(0) === 0xFEFF) cur = cur.slice(1);
const curLines = cur.split(/\r?\n/);

let git = fs.readFileSync(gitClean, 'utf8');
if (git.charCodeAt(0) === 0xFEFF) git = git.slice(1);
const gitLines = git.split(/\r?\n/);

// backup
fs.writeFileSync(root + '/_rec/parameter_before_regfix.h', curLines.join('\r\n'), 'utf8');

// git skeleton map
const bySkel = new Map();
for (const g of gitLines) {
  const k = skel(g);
  if (!k) continue;
  if (!bySkel.has(k)) bySkel.set(k, []);
  bySkel.get(k).push(g);
}

const log = [];
// reconstruct 3
for (const ln of Object.keys(reconstruct)) {
  curLines[ln - 1] = reconstruct[ln];
  log.push(`寃利 L${ln} -> ${reconstruct[ln]}`);
}
// jeom 12 via git skeleton
for (const ln of jeomLines) {
  const k = skel(curLines[ln - 1]);
  const cand = bySkel.get(k);
  if (!cand) { log.push(`占 L${ln} SKIP(git매칭없음): ${curLines[ln-1].trim()}`); continue; }
  const uniq = [...new Set(cand)];
  if (uniq.length !== 1) { log.push(`占 L${ln} SKIP(모호 x${uniq.length})`); continue; }
  curLines[ln - 1] = uniq[0];
  log.push(`占 L${ln} -> ${uniq[0].trim()}`);
}

fs.writeFileSync(target, curLines.join('\r\n'), 'utf8');
console.log(log.join('\n'));
