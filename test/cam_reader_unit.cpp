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