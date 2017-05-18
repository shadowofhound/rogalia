/* global T, dom */

"use strict";

T.help = {
    fight: function(combos) {
        return dom.wrap("", [
            dom.make("h3", "近接戦闘"),
            dom.make("p", "敵を攻撃することでバフが付与されます (ニャは対象外)."),
            dom.make("p", "キャラクターにバフがついていると、ステップのスピードが上昇します。"),
            dom.table(["Name", "Combo", "Description"], combos),
            dom.hr(),
            dom.make("h3", "遠距離攻撃"),
            dom.make("p", "遠隔武器を使用するには、セカンドハンドが空いている必要があります。"),
            dom.make("p", "矢玉は射撃するために使用します。もちろん武器の種類によって異なります。"),
            dom.make("p", "アリーナでは必要ありません支給されます."),
            dom.wrap("", [
                "Every range weapon has:",
                dom.ul([
                    "射程範囲：範囲外のターゲットは攻撃できません。",
                    "有効範囲：この範囲内では100％の精度があります。",
                    "攻撃速度：どのくらい速く射撃できるか",
                    "弾薬タイプ：石、矢、原子など",
                    "飛翔速度：目標に到達する速度",
                ])
            ]),
            dom.make("p", "射程範囲・有効範囲を表示するには、ctrl + shiftを押してください。"),
            dom.make("p", "ターゲットが射撃半径を離れる場合、ターゲットは攻撃を避けることができます"),
            dom.make("p", "射程範囲を超えると回避される可能性が増加します。"),
        ]);
    },
    combos: {
        de: {
            desc: "バフ",
            effect: "+吸収、+シールドブロックチャンス",
        },
        su: {
            desc: "バフ",
            effect: "+ダメージ、+クリティカルチャンス",
        },
        nya: {
            desc: "範囲バフ",
            effect: "Deには+クリティカルチャンス、Suには+吸収",
        },
        ikkyo: {
            desc: "攻撃",
            effect: "タウント、PVP：確率50％　0.5-1.5秒スタン",
        },
        shihonage: {
            desc: "攻撃",
            effect: "追加ダメージ、5秒間の遅れ",
        },
        iriminage: {
            desc: "攻撃",
            effect: "確率20％　2秒間のスタン",
        },
    },
};
