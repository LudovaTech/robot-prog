#include <gtest/gtest.h>

#include "../src/cam_reader.hpp"



// Test: Cas de base avec une séquence complète "b...e"
TEST(CamReader, ExtractLastCompleteSequenceBasicSequence) {
    String input = "abcde";
    String expected = "bcde";
    EXPECT_EQ(String(extractLastCompleteSequence(input)), String(expected));
}

// Test: Séquence avec plusieurs 'b' et 'e', prendre la dernière séquence complète
TEST(CamReader, ExtractLastCompleteSequenceMultipleSequences) {
    String input = "abcde fghbijklmne";
    String expected = "bijklmne";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: Pas de 'b' dans la chaîne
TEST(CamReader, ExtractLastCompleteSequenceNoBCharacter) {
    String input = "acde";
    String expected = "";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: Pas de 'e' dans la chaîne
TEST(CamReader, ExtractLastCompleteSequenceNoECharacter) {
    String input = "abcdf";
    String expected = "";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: 'b' après le dernier 'e'
TEST(CamReader, ExtractLastCompleteSequenceBAfterLastE) {
    String input = "abcdeb";
    String expected = "bcde";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: Chaîne vide
TEST(CamReader, ExtractLastCompleteSequenceEmptyString) {
    String input = "";
    String expected = "";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: Chaîne sans 'b' et 'e'
TEST(CamReader, ExtractLastCompleteSequenceNoBandECharacters) {
    String input = "xyz";
    String expected = "";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: Chaîne contenant uniquement 'b' et 'e'
TEST(CamReader, ExtractLastCompleteSequenceOnlyBandECharacters) {
    String input = "bebebeb";
    String expected = "be";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// Test: 'b' au début et 'e' à la fin
TEST(CamReader, ExtractLastCompleteSequenceBAtStartEAtEnd) {
    String input = "b...e";
    String expected = "b...e";
    EXPECT_EQ(extractLastCompleteSequence(input), expected);
}

// On commence par un test pour vérifier la mise à jour du cache avec une nouvelle position d'ennemi
TEST(ReadAndUpdateCacheTest, UpdatesCacheWithNewValue) {
    Optional<EnemyGoalPos> newEnemyGoalPos(EnemyGoalPos(10, 20));
    auto result = readAndUpdateCache(newEnemyGoalPos);

    EXPECT_TRUE(result.hasValue());
    EXPECT_EQ(result.value().x(), 10);
    EXPECT_EQ(result.value().y(), 20);
}

// On teste ensuite la validité du cache quand la position d'ennemi est optionnelle
TEST(ReadAndUpdateCacheTest, ReturnsValidCacheWhenNoNewValue) {
    _test_set_cacheEnemyGoalPos(Optional<EnemyGoalPos>(EnemyGoalPos(15, 25)));
    _test_set_timeCacheEnemyGoalPos(millis());

    auto result = readAndUpdateCache(Optional<EnemyGoalPos>());

    EXPECT_TRUE(result.hasValue());
    EXPECT_EQ(result.value().x(), 15);
    EXPECT_EQ(result.value().y(), 25);
}

// On teste le cas où le cache est expiré et une nouvelle position d'ennemi n'est pas disponible
TEST(ReadAndUpdateCacheTest, ReturnsEmptyOptionalWhenCacheExpired) {
    _test_set_cacheEnemyGoalPos(Optional<EnemyGoalPos>(EnemyGoalPos(15, 25)));
    _test_set_timeCacheEnemyGoalPos(millis() - timeCache - 1);

    auto result = readAndUpdateCache(Optional<EnemyGoalPos>());

    EXPECT_FALSE(result.hasValue());
}

// On teste le cas où le cache est toujours valide et une nouvelle position d'ennemi n'est pas disponible
TEST(ReadAndUpdateCacheTest, CacheValidWhenNoNewValueAndWithinTimeCache) {
    cacheEnemyGoalPos = Optional<EnemyGoalPos>(EnemyGoalPos(30, 40));
    timeCacheEnemyGoalPos = millis() - timeCache + 10;  // Cache est encore valide

    auto result = readAndUpdateCache(Optional<EnemyGoalPos>());

    EXPECT_TRUE(result.hasValue());
    EXPECT_EQ(result.value().x(), 30);
    EXPECT_EQ(result.value().y(), 40);
}

// On teste le cas où le cache est invalide et une nouvelle position d'ennemi est disponible
TEST(ReadAndUpdateCacheTest, UpdatesCacheWhenCacheExpiredAndNewValueAvailable) {
    cacheEnemyGoalPos = Optional<EnemyGoalPos>(EnemyGoalPos(30, 40));
    timeCacheEnemyGoalPos = millis() - timeCache - 10;

    Optional<EnemyGoalPos> newEnemyGoalPos(EnemyGoalPos(50, 60));
    auto result = readAndUpdateCache(newEnemyGoalPos);

    EXPECT_TRUE(result.hasValue());
    EXPECT_EQ(result.value().x(), 50);
    EXPECT_EQ(result.value().y(), 60);
}